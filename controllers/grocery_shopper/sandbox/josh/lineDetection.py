
import numpy as np
import matplotlib.pyplot as plt

import numpy as np
from scipy.signal import convolve2d
import matplotlib.pyplot as plt



# # plt.imshow(img)
# kernel = np.ones((3,3))/9
# img = convolve2d(img, kernel)
# img[img > 0.5] = 1
# plt.imshow(img)
# plt.show()


decay=1
def filter(img):
    q25, q75 = np.percentile(img, [25, 75])
    IQR = q75 - q25
    Q1 = q25 - 1.5*IQR
    print(len(img <= Q1))
    img[img <= Q1] -= decay
    img = np.clip(img, 0, 1)
    plt.imshow(img)
    plt.show()
    return img

img = np.load("map.npy")
img = filter(img)
img = filter(img)
img = filter(img)
img = filter(img)
img = filter(img)
img = filter(img)
img = filter(img)
