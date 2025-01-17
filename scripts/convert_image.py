from PIL import Image
import numpy as np
from scipy.ndimage import map_coordinates

import math
import sys

# run via: `python convert_image.py <image_filepath> > hex_values`
# outputs one 12-bit rgb hex value per pixel, one ray of pixels per line
image_in = Image.open(sys.argv[1])

def crop_center(pil_img, crop_width, crop_height):
    img_width, img_height = pil_img.size
    return pil_img.crop(((img_width - crop_width) // 2,
                         (img_height - crop_height) // 2,
                         (img_width + crop_width) // 2,
                         (img_height + crop_height) // 2))

def crop_max_square(pil_img):
    crop_size = min(pil_img.size)
    return crop_center(pil_img, crop_size, crop_size)
  
square_image = crop_max_square(image_in)

# TODO: perhaps do an average of 'nearest' mode AND an interpolated value? that way we bias towards nearest, but still get some blending?
def get_pixel_values(image, coords):
  return map_coordinates(image, coords, mode='nearest')

# Convert the image to RGB format
rgb_img = np.array(square_image.convert('RGB'))
# TODO: clean up
r_channel = [[r for r, _, _ in row] for row in rgb_img]
g_channel = [[g for _, g, _ in row] for row in rgb_img]
b_channel = [[b for _, _, b in row] for row in rgb_img]


num_rays = 360 * 3 # 1/3 degrees between each ray
num_pixels = 96
rope_pixels = 25 # number of pixels that would fit between pixel 0 on the rod and the center of spinning
center_px = num_pixels + rope_pixels
scale_factor = square_image.size[0] / (center_px * 2.0)

rads_per_ray = 2 * math.pi / num_rays

# [1080 by 96 by 3] 4-bit ints
rays = [] # store each ray as a list of `num_pixels` 12-bit rgb elements
coordinates = [[], []] # unzipped x,y pairs


for ray in range(0, num_rays):
  ray_pixels = []
  for px in range(0, num_pixels):
    rad = ray * rads_per_ray
    x = (((rope_pixels + px) * math.cos(rad)) + center_px) * scale_factor
    y = (((rope_pixels + px) * math.sin(rad)) + center_px) * scale_factor
    coordinates[0].append(x)
    coordinates[1].append(y)
  rays.append(ray_pixels)

red_pixels = get_pixel_values(r_channel, coordinates)
green_pixels = get_pixel_values(g_channel, coordinates)
blue_pixels = get_pixel_values(b_channel, coordinates)

rgb_pixels = list(zip(red_pixels, green_pixels, blue_pixels))


# div pixel values by 16 to convert to 12-bit color
rgb_pixels_12bit = ["".join([format(v, 'x') for v in [r // 16, g // 16, b // 16]]) for [r, g, b] in rgb_pixels]


ray_pixels_list = [",".join(rgb_pixels_12bit[(ray * num_pixels):((ray + 1) * num_pixels)]) for ray in range(0, num_rays)]


for pxs in ray_pixels_list:
  print(pxs)
