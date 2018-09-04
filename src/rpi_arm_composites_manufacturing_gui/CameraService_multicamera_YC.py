import PySpin
import numpy as np
import cv2.aruco as aruco
import cv2
from tf.transformations import *
#from CameraParams import *
import time
import pickle

def acquire_images(cam, nodemap, nodemap_tldevice):
    """
    :param cam: Camera to acquire images from.
    :param nodemap: Device nodemap.
    :param nodemap_tldevice: Transport layer device nodemap.
    :type cam: CameraPtr
    :type nodemap: INodeMap
    :type nodemap_tldevice: INodeMap
    :return: True if successful, False otherwise.
    :rtype: bool
    """
    NUM_IMAGES = 1  # number of images to grab
    print "*** IMAGE ACQUISITION ***\n"
    try:
        result = True

        # Set acquisition mode to continuous
        #
        #  *** NOTES ***
        #  Because the example acquires and saves 10 images, setting acquisition
        #  mode to continuous lets the example finish. If set to single frame
        #  or multiframe (at a lower number of images), the example would just
        #  hang. This would happen because the example has been written to
        #  acquire 10 images while the camera would have been programmed to
        #  retrieve less than that.
        #
        #  Setting the value of an enumeration node is slightly more complicated
        #  than other node types. Two nodes must be retrieved: first, the
        #  enumeration node is retrieved from the nodemap; and second, the entry
        #  node is retrieved from the enumeration node. The integer value of the
        #  entry node is then set as the new value of the enumeration node.
        #
        #  Notice that both the enumeration and the entry nodes are checked for
        #  availability and readability/writability. Enumeration nodes are
        #  generally readable and writable whereas their entry nodes are only
        #  ever readable.
        #
        #  Retrieve enumeration node from nodemap

        # In order to access the node entries, they have to be casted to a pointer type (CEnumerationPtr here)
        node_acquisition_mode = PySpin.CEnumerationPtr(nodemap.GetNode("AcquisitionMode"))
        if not PySpin.IsAvailable(node_acquisition_mode) or not PySpin.IsWritable(node_acquisition_mode):
            print "Unable to set acquisition mode to continuous (enum retrieval). Aborting..."
            return False

        # Retrieve entry node from enumeration node
        node_acquisition_mode_continuous = node_acquisition_mode.GetEntryByName("Continuous")
        if not PySpin.IsAvailable(node_acquisition_mode_continuous) or not PySpin.IsReadable(node_acquisition_mode_continuous):
            print "Unable to set acquisition mode to continuous (entry retrieval). Aborting..."
            return False

        # Retrieve integer value from entry node
        acquisition_mode_continuous = node_acquisition_mode_continuous.GetValue()

        # Set integer value from entry node as new value of enumeration node
        node_acquisition_mode.SetIntValue(acquisition_mode_continuous)

        print "Acquisition mode set to continuous..."

        #  Begin acquiring images
        #
        #  *** NOTES ***
        #  What happens when the camera begins acquiring images depends on the
        #  acquisition mode. Single frame captures only a single image, multi
        #  frame catures a set number of images, and continuous captures a
        #  continuous stream of images. Because the example calls for the
        #  retrieval of 10 images, continuous mode has been set.
        #
        #  *** LATER ***
        #  Image acquisition must be ended when no more images are needed.
        cam.BeginAcquisition()

        print "Acquiring images..."

        #  Retrieve device serial number for filename
        #
        #  *** NOTES ***
        #  The device serial number is retrieved in order to keep cameras from
        #  overwriting one another. Grabbing image IDs could also accomplish
        #  this.
        device_serial_number = ""
        node_device_serial_number = PySpin.CStringPtr(nodemap_tldevice.GetNode("DeviceSerialNumber"))
        if PySpin.IsAvailable(node_device_serial_number) and PySpin.IsReadable(node_device_serial_number):
            device_serial_number = node_device_serial_number.GetValue()
            print "Device serial number retrieved as %s..." % device_serial_number

        # Retrieve, convert, and save images
        for i in range(NUM_IMAGES):
            try:

                #  Retrieve next received image
                #
                #  *** NOTES ***
                #  Capturing an image houses images on the camera buffer. Trying
                #  to capture an image that does not exist will hang the camera.
                #
                #  *** LATER ***
                #  Once an image from the buffer is saved and/or no longer
                #  needed, the image must be released in order to keep the
                #  buffer from filling up.
                image_result = cam.GetNextImage()

                #  Ensure image completion
                #
                #  *** NOTES ***
                #  Images can easily be checked for completion. This should be
                #  done whenever a complete image is expected or required.
                #  Further, check image status for a little more insight into
                #  why an image is incomplete.
                if image_result.IsIncomplete():
                    print "Image incomplete with image status %d ..." % image_result.GetImageStatus()

                else:

                    #  Print image information; height and width recorded in pixels
                    #
                    #  *** NOTES ***
                    #  Images have quite a bit of available metadata including
                    #  things such as CRC, image status, and offset values, to
                    #  name a few.
                    width = image_result.GetWidth()
                    height = image_result.GetHeight()
                    print "Grabbed Image %d, width = %d, height = %d" % (i, width, height)

                    #  Convert image to mono 8
                    #
                    #  *** NOTES ***
                    #  Images can be converted between pixel formats by using
                    #  the appropriate enumeration value. Unlike the original
                    #  image, the converted one does not need to be released as
                    #  it does not affect the camera buffer.
                    #
                    #  When converting images, color processing algorithm is an
                    #  optional parameter.
                    image_converted = image_result.Convert(PySpin.PixelFormat_Mono8, PySpin.HQ_LINEAR)

                    # Create a unique filename
                    tic = time.time()
                    if device_serial_number:
                        filename = "Acquisition3-%s-%d-%d.jpg" % (device_serial_number, i, tic)
                    else:  # if serial number is empty
                        filename = "Acquisition-%d-%d.jpg" % (i, tic)

                    #  Save image
                    #
                    #  *** NOTES ***
                    #  The standard practice of the examples is to use device
                    #  serial numbers to keep images of one device from
                    #  overwriting those of another.
                    image_result.Save(filename)
                    print "Image saved at %s" % filename

                    #  Release image
                    #
                    #  *** NOTES ***
                    #  Images retrieved directly from the camera (i.e. non-converted
                    #  images) need to be released in order to keep from filling the
                    #  buffer.
                    image_result.Release()
                    print ""
                    #raw_input("Press Enter to continue...")

            except PySpin.SpinnakerException as ex:
                print "Error: %s" % ex
                return False

        #  End acquisition
        #
        #  *** NOTES ***
        #  Ending acquisition appropriately helps ensure that devices clean up
        #  properly and do not need to be power-cycled to maintain integrity.
        cam.EndAcquisition()
    
    except PySpin.SpinnakerException as ex:
        print "Error: %s" % ex
        return False

    return image_converted


def print_device_info(nodemap):
    """
    This function prints the device information of the camera from the transport
    layer; please see NodeMapInfo example for more in-depth comments on printing
    device information from the nodemap.

    :param nodemap: Transport layer device nodemap.
    :type nodemap: INodeMap
    :returns: True if successful, False otherwise.
    :rtype: bool
    """

    print "*** DEVICE INFORMATION ***\n"

    try:
        result = True
        node_device_information = PySpin.CCategoryPtr(nodemap.GetNode("DeviceInformation"))

        if PySpin.IsAvailable(node_device_information) and PySpin.IsReadable(node_device_information):
            features = node_device_information.GetFeatures()
            for feature in features:
                node_feature = PySpin.CValuePtr(feature)
                print "%s: %s" % (node_feature.GetName(),
                                  node_feature.ToString() if PySpin.IsReadable(node_feature) else "Node not readable")

        else:
            print "Device control information not available."

    except PySpin.SpinnakerException as ex:
        print "Error: %s" % ex
        return False

    return result


def run_single_camera(cam):
    """
    This function acts as the body of the example; please see NodeMapInfo example
    for more in-depth comments on setting up cameras.

    :param cam: Camera to run on.
    :type cam: CameraPtr
    :return: True if successful, False otherwise.
    :rtype: bool
    """
    try:
        result = True

        # Retrieve TL device nodemap and print device information
        nodemap_tldevice = cam.GetTLDeviceNodeMap()

        result &= print_device_info(nodemap_tldevice)

        # Initialize camera
        cam.Init()

        # Retrieve GenICam nodemap
        nodemap = cam.GetNodeMap()

        # Acquire images
        result = acquire_images(cam, nodemap, nodemap_tldevice)

        # Deinitialize camera
        cam.DeInit()

    except PySpin.SpinnakerException as ex:
        print "Error: %s" % ex
        result = False

    return result

   
class CameraParams:
    def __init__(self,M00,M02,M11,M12,M22,C00,C01,C02,C03,C04):
        self.camMatrix, self.distCoeff = self.CameraParams(M00,M02,M11,M12,M22,C00,C01,C02,C03,C04)
    
        
    def CameraParams(self, M00,M02,M11,M12,M22,C00,C01,C02,C03,C04):
        camMatrix = np.zeros((3, 3),dtype=np.float64)
        camMatrix[0][0] = M00
        camMatrix[0][2] = M02
        camMatrix[1][1] = M11
        camMatrix[1][2] = M12
        camMatrix[2][2] = M22

        distCoeff = np.zeros((1, 5), dtype=np.float64)
        distCoeff[0][0] = C00
        distCoeff[0][1] = C01
        distCoeff[0][2] = C02
        distCoeff[0][3] = C03
        distCoeff[0][4] = C04
            
#        params = {'camMatrix': camMatrix, 'distCoeff': distCoeff}
        return camMatrix,distCoeff
        

def CameraService():
    """
    Example entry point; please see Enumeration example for more in-depth
    comments on preparing and cleaning up the system.

    :return: True if successful, False otherwise.
    :rtype: bool
    """
    
    # Retrieve singleton reference to system object
    system = PySpin.System.GetInstance()

    # Retrieve list of cameras from the system
    cam_list = system.GetCameras()
    num_cameras = cam_list.GetSize()
    print "Number of cameras detected: %d" % num_cameras

    # Finish if there are no cameras
    if num_cameras == 0:

        # Clear camera list before releasing system
        cam_list.Clear()

        # Release system
        system.ReleaseInstance()

        print "Not enough cameras!"
        raw_input("Done! Press Enter to exit...")
        return False

    # aruco code initialization 
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
    parameters =  cv2.aruco.DetectorParameters_create()
    parameters.cornerRefinementWinSize=32
    parameters.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_CONTOUR
            
    gripper_board=cv2.aruco.GridBoard_create(2, 2, .09792, .00557, aruco_dict, 1)
    panel_board=cv2.aruco.GridBoard_create(2, 2, .09792, .00557, aruco_dict, 5)
    ref1=cv2.aruco.GridBoard_create(4, 1, .062, .0074, aruco_dict, 9)
    ref2=cv2.aruco.GridBoard_create(4, 1, .062, .0074, aruco_dict, 13)
    ref3=cv2.aruco.GridBoard_create(4, 1, .062, .0074, aruco_dict, 17)
    ref4=cv2.aruco.GridBoard_create(4, 1, .062, .0074, aruco_dict, 21)
    tag_ids=["vacuum_gripper_marker_1","leeward_mid_panel_marker_1", "aligner_ref_1", "aligner_ref_2", "aligner_ref_3", "aligner_ref_4"]   
    boards=[gripper_board, panel_board, ref1, ref2, ref3, ref4]   
    
    # Run example on Overhead camera
    #Tca = [[[0,0,0,0],[0,0,0,0],[0,0,0,0]] * 2] * 3


    for i in range(num_cameras):
        
        sn = cam_list.GetByIndex(i).TLDevice.DeviceSerialNumber()
        if sn == '18080264':
             cam = cam_list.GetBySerial('18080264')
             CamParam = CameraParams(5074.887244656572,2651.236256496624, 5066.186708845025, 1744.050554198541, 1.0, -0.1187,0.1812, 0.0, 0.0, 0.0)
        elif sn == '18285636':
             cam = cam_list.GetBySerial('18285636')
             #CamParam = CameraParams(2446.884183720913,1259.372262745234, 2430.897446945758, 1130.106905189156, 1.0,-0.024675346066855, 0.003626674297774, 0.0, 0.0, 0.0)
             CamParam = CameraParams(2282.523358266698,1219.470092884672, 2280.155828279608, 1047.544922259821, 1.0,-0.026551785846910,-0.024902017224009, 0.0, 0.0, 0.0)
        elif sn == '18285621':
             cam = cam_list.GetBySerial('18285621')
             #CamParam = CameraParams(2255.146571049155,1242.868511771508, 2254.856700048628, 1013.386203214607, 1.0,-0.021819685007776, -0.011049716197787, 0.0, 0.0, 0.0)
             CamParam = CameraParams(2289.766897623516,1208.730006299109, 2287.118412509352, 1046.441859706861, 1.0,-0.028273291300099, -0.017090212502593, 0.0, 0.0, 0.0)   

        
        print "Running example for camera %d..." % i
        result = run_single_camera(cam)
        print "Camera %d example complete..." % i
        # Operations on the frame
        if result != False:
            
            frame = np.array(result.GetData(), dtype="uint8").reshape( (result.GetHeight(), result.GetWidth(),1))                                 
            corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=parameters)
            
            for board,tag_id in zip(boards,tag_ids):            
            
                retval, rvec, tvec = cv2.aruco.estimatePoseBoard(corners, ids, board, CamParam.camMatrix, CamParam.distCoeff)
                if (retval > 0):
                    print "tag_id "+tag_id
                    #print rvec, tvec
                       
                    Ra, b = cv2.Rodrigues(rvec)
                    tmp = np.hstack([Ra,tvec])
                    
                    '''
                    filename = "Tca_%s_%s.txt" % (sn,tag_id)
                    f_handle = file(filename, 'a')
                    np.savetxt(f_handle, tmp)
                    f_handle.close()
                    '''

                    if tag_id== "aligner_ref_1":
                        
                        if sn == '18285636':
                            Tca1 = np.vstack([tmp,[0.0,0.0,0.0,1.0]])  
                            print 'Save Tag 1 data of camera',sn
                            print Tca1
                        
                    elif tag_id== "aligner_ref_2":
                        
                        if sn == '18285636':
                            Tca2 = np.vstack([tmp,[0.0,0.0,0.0,1.0]])   
                            print 'Save Tag 2 data of camera',sn
                            print Tca2

        del cam



    # Clear camera list before releasing system
    cam_list.Clear()

    # Release instance
    system.ReleaseInstance()

    #raw_input("Done! Press Enter to exit...")
    #return Pca, Rca
    return Tca1,Tca2

if __name__ == "__main__":
    CameraService()
