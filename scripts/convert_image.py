from PIL import Image
import numpy as np
from scipy.ndimage import map_coordinates

import math

image = Image.open('/Users/eddie/spinnystick/images/display/mandala1.jpg')

def crop_center(pil_img, crop_width, crop_height):
    img_width, img_height = pil_img.size
    return pil_img.crop(((img_width - crop_width) // 2,
                         (img_height - crop_height) // 2,
                         (img_width + crop_width) // 2,
                         (img_height + crop_height) // 2))

def crop_max_square(pil_img):
    crop_size = min(pil_img.size)
    return crop_center(pil_img, crop_size, crop_size)
  
square_image = crop_max_square(image)

# Gets the pixel value at fractional coordinates (x, y) using bilinear interpolation
def get_pixel_value(image, x, y):
  return map_coordinates(image, [[y], [x]], order=1)[0]

# Convert the image to RGB format
rgb_img = square_image.convert('RGB')


num_rays = 360 * 3 # 0.3 degrees between each ray
num_pixels = 96
rope_pixels = 45 # number of pixels that would fit between pixel 0 on the rod and the center of spinning
center_px = num_pixels + rope_pixels

rads_per_ray = 2 * math.pi / num_rays

# [1080 by 96 by 3] 4-bit ints
rays = [] # store each ray as a list of `num_pixels` 12-bit rgb elements

for ray in range(0, num_rays):
  rays[ray] = []
  for px in range(0, num_pixels):
    rad = ray * rads_per_ray
    x = ((rope_pixels + px) * math.cos(rad)) + center_px
    y = ((rope_pixels + px) * math.sin(rad)) + center_px
    [r, g, b] = get_pixel_value(rgb_img, x, y)
    rays[ray].append([r / 2, g / 2, b / 2]) # div by 2 for 12-bit color (that works right?)


hex_values = [['#{:1x}{:1x}{:1x}'.format(r, g, b) for r, g, b in ray] for ray in rays]

print(hex_values)
