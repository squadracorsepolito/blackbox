from PIL import Image

def to_rgb565(r, g, b):
    r = (r & 0b11111000) << 8
    g = (g & 0b11111100) << 3
    b = b >> 3
    val = r + g + b
    return val.to_bytes(2, 'big')


def im_to_raw(im):
    raw = []
    for y in range(im.height):
        for x in range(im.width):
            r, g, b, _ = im.getpixel((x, y))
            raw.append(to_rgb565(r, g, b))
    return b''.join(raw)


im = Image.open("meme.png")

assert im.width <= 160
assert im.height <= 80

with open("meme.raw", "wb") as fp:
    raw = im_to_raw(im)
    fp.write(raw)