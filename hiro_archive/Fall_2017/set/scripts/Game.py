import sys

from Turn import *
from opencv import *

"""
Authors: Cassandra Overney and Enmo Ren
Purpose: Integrates the opencv, Set and Turn files. Basically takes an image and returns a set from it with the cards coordinates

"""

image = cv2.imread("test106.jpg")
print image
try:
    ceo = CEO()
    results = ceo.find_matches(im=image)
except:
    print("something is wrong, take another picture")
    sys.exit(0)
# print("final", results)

turn1 = Turn(results)
result = turn1.find_set()
turn1.print_card_array(turn1.card_array)

cv2.imshow('Image', image)
cv2.waitKey(0)
cv2.destroyAllWindows()
