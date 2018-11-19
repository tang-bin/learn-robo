import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only


def color_thresh(img, rgb_thresh=(190, 150, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:, :, 0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:, :, 0] > rgb_thresh[0]) & (img[:, :, 1] > rgb_thresh[1]) & (img[:, :, 2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

# Define a function to convert from image coords to rover coords


def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the
    # center bottom of the image.
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle)
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to map rover space pixels to world space

def rotate(image, angle, center=None, scale=1.0):
    (h, w) = image.shape[:2]
    if center is None:
        center = (w // 2, h // 2)

    M = cv2.getRotationMatrix2D(center, angle, scale)

    rotated = cv2.warpAffine(image, M, (w, h))
    return rotated #7

def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))

    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result
    return xpix_rotated, ypix_rotated


def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale):
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, worldSize, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, worldSize - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, worldSize - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform


def perspect_transform(img, src, dst, roll, pitch):

    h, w = img.shape[:2]
    center = (w // 2, h - 20)
    size = (w, h)
    
    M = cv2.getRotationMatrix2D(center, -roll, 1.0)
    #img = cv2.warpAffine(img, M, (w, h))

    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, size)
    mask = cv2.warpPerspective(np.ones_like(img[:, :, 0]), M, size)
    return warped, mask


def findRocks(img, levels=(130, 110, 100)):
    rockPix = (img[:, :, 0] > levels[0]) & (img[:, :, 1] > levels[1]) & (img[:, :, 2] < levels[2])
    colorSelect = np.zeros_like(img[:, :, 0])
    colorSelect[rockPix] = 1
    return colorSelect

# Apply the above functions in succession and update the Rover state accordingly

def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO:
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    src = np.float32([[14, 140], [301, 140], [200, 96], [118, 96]])
    sq = 5
    offset = 5
    xx = Rover.img.shape[0]
    yy = Rover.img.shape[1]
    dest = np.float32([[yy/2 - sq, xx - offset], [yy/2 + sq, xx - offset],
                       [yy/2 + sq, xx - 2 * sq - offset], [yy/2 - sq, xx - 2 * sq - offset]])

    # 2) Apply perspective transform

    wrapedImg, mask = perspect_transform(Rover.img, src, dest, Rover.roll, Rover.pitch)

    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples

    threshold = color_thresh(wrapedImg)
    obsMap = np.absolute(np.float32(threshold) - 1) * mask

    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
    # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
    #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
    #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image

    worldSize = Rover.worldmap.shape[0]
    scale = 2 * sq

    rockMap = findRocks(wrapedImg)
    if rockMap.any():
        rockX, rockY = rover_coords(rockMap)
        rockWorldX, rockWorldY = pix_to_world(rockX, rockY, Rover.pos[0], Rover.pos[1], Rover.yaw, worldSize, scale)
        rockDist, rockAng = to_polar_coords(rockX, rockY)
        rockIndex = np.argmin(rockDist)
        rockWorldMapX = rockWorldX[rockIndex]
        rockWorldMapY = rockWorldY[rockIndex]

        Rover.worldmap[rockWorldMapY, rockWorldMapX, 1] = 255 # add Green to map (
        Rover.vision_image[:, :, 1] = rockMap * 255
    else:
        Rover.vision_image[:, :, 1] = 0

    
    rollThreshold = 0.3
    pitchThreshold = 0.3 


    updateWorldMap = (Rover.roll <= rollThreshold or Rover.roll >= 360 - rollThreshold) and (Rover.pitch <= pitchThreshold or Rover.pitch >= 360 - pitchThreshold)

    print("Update ? ", Rover.roll, Rover.pitch, updateWorldMap)

    Rover.vision_image[:, :, 2] = threshold * 255
    roverX, roverY= rover_coords(threshold)
    roverWorldX, roverWorldY = pix_to_world(roverX, roverY, Rover.pos[0], Rover.pos[1], Rover.yaw, worldSize, scale)
    if updateWorldMap:
        Rover.worldmap[roverWorldY, roverWorldX, 2] += 5

    Rover.vision_image[:, :, 0] = obsMap * 255
    obsRoverX, obsRoverY = rover_coords(obsMap)
    obsWorldX, obsWorldY= pix_to_world(obsRoverX, obsRoverY, Rover.pos[0], Rover.pos[1], Rover.yaw, worldSize, scale)
    if updateWorldMap:
        Rover.worldmap[obsWorldY, obsWorldX, 0] += 5

    # 7) Update Rover worldmap (to be displayed on right side of screen)
    # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
    #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
    #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1


    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
    # Rover.nav_dists = rover_centric_pixel_distances
    # Rover.nav_angles = rover_centric_angles



    dist, angle = to_polar_coords(roverX, roverY)
    Rover.nav_angles = angle

    return Rover
