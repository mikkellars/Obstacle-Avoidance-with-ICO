"""Document for trimming down the ada20k dataset
"""
from shutil import copyfile
from tqdm import tqdm

def sort_scene_categories(data_path:str, want_classes):
    want_imgs = []

    path_scene_cat = data_path + "/sceneCategories.txt"

    with open(path_scene_cat, "r") as f:
        for line in f:
            img, cl = line.split(" ")
            cl = cl.replace("\n", "")
            if cl in want_classes:
                want_imgs.append(img)

    return want_imgs


def trim_dataset(data_path:str, save_path:str, trimmed_imgs:list):
    img_path_load = data_path + "/images/"
    mask_path_load = data_path + "/annotations/"

    img_path_save = save_path + "/images/"
    mask_path_save = save_path + "/annotations/"

    loop = tqdm(trimmed_imgs)
    for img_name in loop:
        if "train" in img_name:
            copyfile(img_path_load + "/training/" + img_name + ".jpg", img_path_save + "/training/" + img_name + ".jpg") # Copy images
            copyfile(mask_path_load + "/training/" + img_name + ".png", mask_path_save + "/training/" + img_name + ".png") # Copy masks
        elif "val" in img_name:
            copyfile(img_path_load + "/validation/" + img_name + ".jpg", img_path_save + "/validation/" + img_name + ".jpg") # Copy images
            copyfile(mask_path_load + "/validation/" + img_name + ".png", mask_path_save + "/validation/" + img_name + ".png") # Copy masks





data_path = "/home/mikkel/Documents/data/ADEChallengeData2016"
save_path = "/home/mikkel/Documents/data/ADETrimmed"

want_classes = ["corridor", "box", "conference_room", "living_room", "alcove", "cabin_indoor"]

trim_imgs = sort_scene_categories(data_path, want_classes)
trim_dataset(data_path, save_path, trim_imgs)
