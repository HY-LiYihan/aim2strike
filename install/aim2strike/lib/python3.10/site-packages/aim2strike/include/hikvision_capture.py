# -- coding: utf-8 --

import sys
import ctypes
import numpy as np
import time
import os
import threading # Keep for potential future extensions like threaded reading

# --- HIKVISION SDK Import ---
# try:
#     # Adjust this path if your MVS SDK installation differs
#     # Or ensure the path is added to your system's PYTHONPATH environment variable
#     sys.path.append(os.getcwd()+"/src/MvImport") # Example path
#     from src.MvImport.MvCameraControl_class import *
# except ImportError as e:
#     print(f"Error: MvCameraControl_class not found. {e}")
#     print("Please ensure the MVS SDK Python samples path is correct or in PYTHONPATH.")
#     print("Install the Hikvision MVS SDK and its Python wrapper.")
#     sys.exit()
# except Exception as e:
#     print(f"An unexpected error occurred during MVS SDK import: {e}")
#     sys.exit()

from aim2strike.MvImport.MvCameraControl_class import *

class HikvisionCameraError(Exception):
    """Custom exception for Hikvision Camera errors."""
    pass


class HikvisionCamera:
    """
    A class to interact with Hikvision industrial cameras using the MVS SDK,
    providing an interface similar to OpenCV's VideoCapture.

    Requires the Hikvision MVS SDK and its Python wrapper to be installed
    and accessible.

    Usage:
        # Initialize SDK *once* before creating any camera instance
        HikvisionCamera.initialize_sdk()

        try:
            # Use a context manager for automatic resource release
            with HikvisionCamera(device_index=0) as cam:
                if not cam.is_open():
                    print("Camera failed to open.")
                    exit()

                print(f"Camera opened: {cam.get_resolution()}")
                cam.start()

                while True:
                    ret, frame = cam.read()
                    if not ret:
                        # Handle timeout or error
                        print("Warning: Failed to grab frame")
                        # Optional: add a small sleep or break
                        time.sleep(0.01)
                        continue

                    # --- Process frame (e.g., display) ---
                    import cv2 # Import cv2 only if needed for display/processing
                    cv2.imshow('Hikvision Feed', frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break

                cam.stop() # Optional, called by __exit__ anyway

        except HikvisionCameraError as e:
            print(f"Camera Error: {e}")
        except Exception as e:
            print(f"An unexpected error occurred: {e}")
        finally:
            # Finalize SDK *once* after all camera instances are released
            HikvisionCamera.finalize_sdk()
            # Optional: destroy cv2 windows if used
            try:
                import cv2
                cv2.destroyAllWindows()
            except ImportError:
                pass # cv2 wasn't used
            print("Program finished.")

    """
    _sdk_initialized = False
    _lock = threading.Lock() # Protect SDK init/finalize if used across threads

    def __init__(self, device_index=0, verbose=False):
        """
        Initializes the camera instance.

        Args:
            device_index (int): The index of the camera to open if multiple are found.
            verbose (bool): Print more detailed information during setup.

        Raises:
            HikvisionCameraError: If SDK is not initialized or camera setup fails.
        """
        if not HikvisionCamera._sdk_initialized:
            raise HikvisionCameraError("SDK not initialized. Call HikvisionCamera.initialize_sdk() first.")

        self.cam = None
        self.st_device_info = None
        self.width = 0
        self.height = 0
        self.p_converted_data = None
        self._buffer_size = 0
        self._is_grabbing = False
        self._is_open = False
        self._verbose = verbose

        try:
            # --- Enumerate Devices ---
            device_list = MV_CC_DEVICE_INFO_LIST()
            # Add other types like MV_CAMERALINK_DEVICE if needed
            tlayer_type = MV_GIGE_DEVICE | MV_USB_DEVICE
            ret = MvCamera.MV_CC_EnumDevices(tlayer_type, device_list)
            if ret != 0:
                raise HikvisionCameraError(f"Enum devices failed ({ret:#x})")
            if device_list.nDeviceNum == 0:
                raise HikvisionCameraError("No compatible Hikvision devices found.")

            if self._verbose:
                print(f"Found {device_list.nDeviceNum} devices.")
                # Optionally list devices here

            if not (0 <= device_index < device_list.nDeviceNum):
                 raise HikvisionCameraError(f"Invalid device_index ({device_index}). "
                                           f"Must be between 0 and {device_list.nDeviceNum - 1}.")

            # --- Create Camera Instance & Handle ---
            self.cam = MvCamera()
            self.st_device_info = cast(device_list.pDeviceInfo[device_index],
                                       POINTER(MV_CC_DEVICE_INFO)).contents
            ret = self.cam.MV_CC_CreateHandle(self.st_device_info)
            if ret != 0:
                self.cam = None # Ensure cam is None if handle creation failed
                raise HikvisionCameraError(f"Create handle fail! ret[0x{ret:x}]")

            # --- Open Device ---
            ret = self.cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
            if ret != 0:
                self._clean_handle()
                raise HikvisionCameraError(f"Open device fail! ret[0x{ret:x}]")

            self._is_open = True
            if self._verbose:
                print(f"Camera device {device_index} opened successfully.")

            # --- Optimize Packet Size (GigE Only) ---
            if self.st_device_info.nTLayerType == MV_GIGE_DEVICE:
                n_packet_size = self.cam.MV_CC_GetOptimalPacketSize()
                if n_packet_size > 0:
                    ret = self.cam.MV_CC_SetIntValue("GevSCPSPacketSize", n_packet_size)
                    if ret != 0:
                        print(f"Warning: Set Packet Size fail! ret[0x{ret:x}]")
                    elif self._verbose:
                        print(f"Optimal packet size set to {n_packet_size}.")
                else:
                    print(f"Warning: Get Optimal Packet Size fail! ret[0x{n_packet_size:x}]")

            # --- Set Trigger Mode Off (Continuous Acquisition) ---
            ret = self.cam.MV_CC_SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF)
            if ret != 0:
                print(f"Warning: Set trigger mode off fail! ret[0x{ret:x}]")

            # --- Get Camera Resolution ---
            st_width = MVCC_INTVALUE_EX()
            st_height = MVCC_INTVALUE_EX()
            ret = self.cam.MV_CC_GetIntValueEx("Width", st_width)
            if ret != 0: raise HikvisionCameraError(f"Get Width failed ({ret:#x})")
            ret = self.cam.MV_CC_GetIntValueEx("Height", st_height)
            if ret != 0: raise HikvisionCameraError(f"Get Height failed ({ret:#x})")
            self.width = st_width.nCurValue
            self.height = st_height.nCurValue
            if self._verbose:
                print(f"Camera resolution: {self.width} x {self.height}")

            # --- Allocate Conversion Buffer (Target: BGR8 for OpenCV) ---
            self._buffer_size = self.width * self.height * 3 # BGR requires 3 bytes per pixel
            # Ensure buffer size is reasonable
            if self._buffer_size <= 0:
                 raise HikvisionCameraError("Invalid buffer size calculated (width or height is zero).")
            # Use ctypes.create_string_buffer for mutable byte buffer
            # self.p_converted_data = (ctypes.c_ubyte * self._buffer_size)()
            self.p_converted_data = ctypes.create_string_buffer(self._buffer_size)


        except Exception as e:
            # Ensure cleanup happens if initialization fails midway
            self.release()
            # Re-raise the exception or a specific camera error
            if isinstance(e, HikvisionCameraError):
                raise e
            else:
                 # Wrap unexpected errors
                raise HikvisionCameraError(f"Unexpected error during camera initialization: {e}") from e

    @staticmethod
    def initialize_sdk():
        """Initializes the MVS SDK. Call this *once* before creating any HikvisionCamera."""
        with HikvisionCamera._lock:
            if not HikvisionCamera._sdk_initialized:
                ret = MvCamera.MV_CC_Initialize()
                if ret != 0:
                    raise HikvisionCameraError(f"MVS SDK Initialize fail! ret[0x{ret:x}]")
                HikvisionCamera._sdk_initialized = True
                print("MVS SDK Initialized.")
            else:
                print("MVS SDK already initialized.")

    @staticmethod
    def finalize_sdk():
        """Finalizes the MVS SDK. Call this *once* after all HikvisionCamera instances are released."""
        with HikvisionCamera._lock:
            if HikvisionCamera._sdk_initialized:
                ret = MvCamera.MV_CC_Finalize()
                if ret != 0:
                    # Log error but don't raise exception during cleanup ideally
                    print(f"Error: MVS SDK Finalize fail! ret[0x{ret:x}]")
                HikvisionCamera._sdk_initialized = False
                print("MVS SDK Finalized.")
            # else:
            #     print("MVS SDK not initialized or already finalized.")


    def is_open(self):
        """Check if the camera connection is established."""
        # Check both the internal flag and if the handle is valid
        return self._is_open and self.cam is not None and self.cam.handle is not None

    def start(self):
        """Starts the camera's frame grabbing."""
        if not self.is_open():
            print("Warning: Cannot start grabbing, camera is not open.")
            return False
        if self._is_grabbing:
            print("Warning: Camera is already grabbing.")
            return True # Already started

        ret = self.cam.MV_CC_StartGrabbing()
        if ret != 0:
            print(f"Error: Start grabbing fail! ret[0x{ret:x}]")
            self._is_grabbing = False
            return False
        self._is_grabbing = True
        if self._verbose:
            print("Camera stream started.")
        return True

    def stop(self):
        """Stops the camera's frame grabbing."""
        if not self.is_open():
            # No need to stop if not open
            return True
        if not self._is_grabbing:
            # Already stopped or never started
            return True

        ret = self.cam.MV_CC_StopGrabbing()
        if ret != 0:
            print(f"Error: Stop grabbing fail! ret[0x{ret:x}]")
            # Continue cleanup even if stop fails
        self._is_grabbing = False
        if self._verbose:
            print("Camera stream stopped.")
        return ret == 0


    def read(self, timeout_ms=1000):
        """
        Grabs, converts, and returns the next frame from the camera.

        Args:
            timeout_ms (int): Maximum time in milliseconds to wait for a frame.

        Returns:
            tuple: (bool, numpy.ndarray or None)
                   - bool: True if a frame was successfully retrieved, False otherwise.
                   - numpy.ndarray: The captured frame as a BGR NumPy array if successful,
                                    None if retrieval failed or timed out.
        """
        if not self._is_grabbing:
            # print("Warning: Camera is not grabbing. Call start() first.")
            return False, None
        if not self.is_open():
            print("Warning: Cannot read frame, camera is not open.")
            return False, None

        st_frame_info = MV_FRAME_OUT()
        memset(byref(st_frame_info), 0, sizeof(st_frame_info))

        # --- Get Raw Frame ---
        ret = self.cam.MV_CC_GetImageBuffer(st_frame_info, timeout_ms)

        if ret == 0:
            # Check if buffer address is valid
            if st_frame_info.pBufAddr is None or st_frame_info.stFrameInfo.nFrameLen == 0:
                print("Warning: GetImageBuffer success but pBufAddr is None or FrameLen is 0")
                # Free buffer even if empty? SDK docs suggest yes for paired calls
                if st_frame_info.pBufAddr:
                     self.cam.MV_CC_FreeImageBuffer(st_frame_info)
                return False, None

            # --- Convert to BGR ---
            st_convert_param = MV_CC_PIXEL_CONVERT_PARAM_EX()
            memset(byref(st_convert_param), 0, sizeof(st_convert_param))
            st_convert_param.nWidth = self.width # Use stored width/height
            st_convert_param.nHeight = self.height
            st_convert_param.pSrcData = st_frame_info.pBufAddr
            st_convert_param.nSrcDataLen = st_frame_info.stFrameInfo.nFrameLen
            st_convert_param.enSrcPixelType = st_frame_info.stFrameInfo.enPixelType
            st_convert_param.enDstPixelType = PixelType_Gvsp_BGR8_Packed # Target OpenCV format
            st_convert_param.pDstBuffer = ctypes.cast(self.p_converted_data, ctypes.POINTER(ctypes.c_ubyte))
            st_convert_param.nDstBufferSize = self._buffer_size

            convert_ret = self.cam.MV_CC_ConvertPixelTypeEx(st_convert_param)

            # --- Free SDK Buffer (Important!) ---
            # Must be freed regardless of conversion success IF GetImageBuffer succeeded
            free_ret = self.cam.MV_CC_FreeImageBuffer(st_frame_info)
            # if free_ret != 0: print(f"Warning: FreeImageBuffer failed ({free_ret:#x})")

            if convert_ret != 0:
                print(f"Error: ConvertPixelTypeEx failed ({convert_ret:#x}) "
                      f"from pixel type {st_frame_info.stFrameInfo.enPixelType}")
                return False, None

            # --- Create NumPy Array ---
            try:
                # Use the destination length from the conversion result
                img_buff = np.frombuffer(self.p_converted_data.raw, dtype=np.uint8, count=st_convert_param.nDstLen)
                # Reshape (ensure size matches expected BGR size)
                expected_size = self.height * self.width * 3
                if img_buff.size == expected_size:
                    frame_bgr = img_buff.reshape((self.height, self.width, 3))
                    return True, frame_bgr.copy() # Return a copy to avoid issues if buffer is reused quickly
                else:
                    print(f"Warning: Converted buffer size mismatch. Got {img_buff.size}, expected {expected_size}")
                    return False, None
            except Exception as e:
                 print(f"Error creating NumPy array from buffer: {e}")
                 return False, None

        # elif ret == MV_E_TIMEOUT:
        #     # Expected case when no new frame is ready within the timeout
        #     # print("Timeout getting image buffer.")
        #     return False, None
        elif ret == MV_E_NODATA:
             print("Warning: MV_E_NODATA received.")
             return False, None
        else:
            # Other potentially critical errors
            print(f"Error: GetImageBuffer failed! ret[0x{ret:x}]")
            # Consider setting an error state or stopping grabbing on critical errors
            # self.stop()
            return False, None

    def get_resolution(self):
        """Returns the camera resolution (width, height)."""
        return self.width, self.height

    def release(self):
        """Releases camera resources (stops grabbing, closes device, destroys handle)."""
        if self._verbose:
            print("Releasing camera resources...")

        if self.cam is not None:
            try:
                if self._is_grabbing:
                    self.stop()
            except Exception as e:
                 print(f"Exception during stop in release: {e}") # Log but continue cleanup

            try:
                 if self._is_open:
                    ret = self.cam.MV_CC_CloseDevice()
                    if ret != 0: print(f"Error closing device: {ret:#x}")
                    self._is_open = False
            except Exception as e:
                 print(f"Exception during close device in release: {e}")

            try:
                 self._clean_handle()
            except Exception as e:
                 print(f"Exception during destroy handle in release: {e}")

        # Clear internal state
        self.cam = None
        self._is_grabbing = False
        self._is_open = False
        self.p_converted_data = None # Allow buffer to be garbage collected
        if self._verbose:
            print("Camera resources released.")


    def _clean_handle(self):
        """Destroys the camera handle if it exists."""
        if self.cam is not None:
             handle_ptr = ctypes.c_void_p(self.cam.handle)
             if handle_ptr: # Check if handle is not NULL
                 ret = self.cam.MV_CC_DestroyHandle()
                 if ret != 0: print(f"Error destroying handle: {ret:#x}")
             self.cam.handle = None # Ensure handle is marked as None
             self.cam = None # Release camera object reference


    # --- Context Manager ---
    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        # This ensures release() is called when exiting the 'with' block
        self.release()

    # --- Destructor (Fallback) ---
    def __del__(self):
        # Best effort cleanup if release() wasn't explicitly called
        # Note: __del__ behavior can be unpredictable, explicit release() or context manager is preferred.
        if self.is_open():
             print("Warning: HikvisionCamera object deleted without explicit release(). Cleaning up...")
             self.release()


# --- Example Usage ---
if __name__ == "__main__":
    # --- IMPORTANT ---
    # Initialize SDK *once* at the start of your application
    try:
        HikvisionCamera.initialize_sdk()
    except HikvisionCameraError as e:
        print(e)
        sys.exit()

    capture = None # Initialize to None
    try:
        # Select camera index 0, enable verbose messages
        capture = HikvisionCamera(device_index=0, verbose=True)

        # Alternatively, use the context manager for guaranteed release:
        # with HikvisionCamera(device_index=0, verbose=True) as capture:
        #     # ... rest of the code indented under 'with' ...
        #     # capture.release() will be called automatically on exit

        if not capture.is_open():
             print("Failed to open camera.")
             # SDK Finalization will happen in finally block
             sys.exit()

        width, height = capture.get_resolution()
        print(f"Successfully opened camera with resolution: {width}x{height}")

        if not capture.start():
            print("Failed to start grabbing.")
            # release() and SDK Finalization will happen in finally block
            sys.exit()

        # --- Import OpenCV here if needed for display ---
        try:
            import cv2
            use_opencv = True
            cv2.namedWindow('Hikvision Feed', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('Hikvision Feed', width // 2, height // 2) # Optional resize
        except ImportError:
            print("OpenCV not found. Frames will not be displayed.")
            use_opencv = False


        # --- Main Loop ---
        frame_count = 0
        start_time = time.time()
        while True:
            ret, frame = capture.read(timeout_ms=1000) # Use the read method

            if not ret:
                # Handle timeout or error - potentially wait briefly
                # print("No frame received.") # Can be noisy
                # time.sleep(0.005) # Small sleep to prevent busy-waiting on timeout
                # Check if camera is still supposed to be grabbing
                if not capture._is_grabbing:
                     print("Grabbing stopped unexpectedly.")
                     break
                continue # Try reading again

            frame_count += 1

            # --- Your image processing code would go here ---
            # Example: detector.preprocess_image(frame), etc.
            # Pass 'frame' (the BGR numpy array) to your detector


            # --- Display Frame (Optional) ---
            if use_opencv:
                # Calculate FPS (optional)
                if frame_count % 30 == 0: # Update FPS every 30 frames
                    end_time = time.time()
                    fps = 30 / (end_time - start_time)
                    start_time = end_time
                    cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

                cv2.imshow('Hikvision Feed', frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    print("Exit key pressed.")
                    break
                # Add other key controls if needed (like 'c' for color)
            else:
                # If not displaying, maybe print frame count or add a delay
                if frame_count % 100 == 0:
                    print(f"Processed {frame_count} frames.")
                # time.sleep(0.01) # Avoid running loop too fast without display delay


    except HikvisionCameraError as e:
        print(f"Camera Error: {e}")
    except KeyboardInterrupt:
         print("Interrupted by user (Ctrl+C).")
    except Exception as e:
        print(f"An unexpected error occurred in the main loop: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # --- Cleanup ---
        print("Starting cleanup...")
        if capture is not None:
            # Explicitly release resources if not using 'with' statement
            capture.release()

        # --- IMPORTANT ---
        # Finalize SDK *once* at the end of your application
        HikvisionCamera.finalize_sdk()

        # Optional: Clean up OpenCV windows if they were used
        if 'use_opencv' in locals() and use_opencv:
            try:
                import cv2
                cv2.destroyAllWindows()
                print("OpenCV windows closed.")
            except ImportError:
                 pass # Should not happen if use_opencv is True, but safe check

        print("Program finished.")