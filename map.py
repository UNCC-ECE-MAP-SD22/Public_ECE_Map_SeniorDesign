from matplotlib import pyplot as plt
from matplotlib import image as mpimg

img = mpimg.imread('UNCEngineerMap.png')
imgplot = plt.imshow(img)
plt.show()