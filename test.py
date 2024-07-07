def make_color_dds(size, color, name):
    import imageio
    from PIL import Image
    img_size = size
    image = Image.new("RGB", img_size, color)
    imageio.imwrite(name+".dds", image, format='DDS')

make_color_dds((512,512), (255,0,0), "Kerb1")