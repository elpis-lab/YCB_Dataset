"""Used to compress high-resolution PNG image to non-spotable compresssed JPG image"""

from PIL import Image
import os


def compress_image(
    origin, target, quality=90, scale=1, resample=Image.LANCZOS
):
    """Load in PNG image and compress it to JPG image"""
    assert 0 < scale <= 1

    # Load image
    im = Image.open(origin).convert("RGBA")
    x, y = im.size
    bg = Image.new("RGBA", (x, y), (255, 255, 255))
    bg.paste(im, (0, 0), im)
    bg = bg.convert("RGB")

    # Scale if necessary
    if scale != 1:
        new_x = max(1, int(round(x * scale)))
        new_y = max(1, int(round(y * scale)))
        bg = bg.resize((new_x, new_y), resample=resample)

    bg.save(target, optimize=True, quality=quality)


# Compress PNG texture of the entire library
# Loop for subfolders
del_prev_img = False
root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
object_folder = "ycb"

curr_dir = root_dir + "/" + object_folder + "/"
for subdir in os.listdir(curr_dir):
    curr_path = os.path.join(curr_dir, subdir)
    if not os.path.isdir(curr_path):
        continue

    file_name = curr_path + "/texture_map.png"
    target = curr_path + "/texture_map.png"  # can also be jpg
    if not os.path.exists(file_name):
        continue

    print("Compressing", subdir)
    # compress_image(file_name, target, quality=80)  # jpg
    compress_image(file_name, target, scale=0.25)  # png

    if del_prev_img and file_name != target:
        os.remove(file_name)
