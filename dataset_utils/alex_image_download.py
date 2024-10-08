from bing_image_downloader import downloader
from PIL import Image
import glob
import os

query = "street level view"
num = 500

##downloader.download(query, limit=num,
##                    output_dir='dataset',
##                    adult_filter_off=False,
##                    force_replace=False,
##                    timeout=10)

for j in range(1,num+1):
    try:
        name = os.path.join(os.path.curdir,
                            'dataset', query, "Image_"+str(j))
        name = glob.glob(name+'.*')[0]
        image = Image.open(name)
        image = image.resize((128,128))
        image.save(name)
        print(j)
    except:
        with open("errors.txt", "a") as errfile:
            errfile.write("issue with "+str(j)+'\n')
