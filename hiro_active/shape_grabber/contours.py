import numpy as np
import imutils
from imutils import perspective
from scipy.spatial import distance as dist
import cv2
import argparse


def midpoint(ptA, ptB):
    return (ptA[0] + ptB[0]) * 0.5, (ptA[1] + ptB[1]) * 0.5


def show_cv(word, draw):
    cv2.imshow(word, draw)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def find_center(cnt, img):
    centers = []
    for i,c in enumerate(cnt):
        # compute the center of the contour
        M = cv2.moments(c)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            centers.append((cX,cY))
            # draw the contour and center of the shape on the image
            cv2.drawContours(img, [c], -1, (0, 255, 0), 2)
            cv2.circle(img, (cX, cY), 1, (255, 255, 255), -1)
            cv2.putText(img, "c %s" % i, (cX - 20, cY - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    # show the image
    # show_cv('center', img)
    return centers

def find_size(cnt, orig):
    ap = argparse.ArgumentParser()
    ap.add_argument("-w", "--width", type=float, required=True,
                    help="width of the left-most object in the image (in inches)")
    args = vars(ap.parse_args())

    pixelsPerMetric = None
    sizes = []

    for i,c in enumerate(cnt):
        # if the contour is not sufficiently large, ignore it
        if cv2.contourArea(c) < 100:
            continue

        # compute the rotated bounding box of the contour
        box = cv2.minAreaRect(c)
        box = cv2.cv.BoxPoints(box) if imutils.is_cv2() else cv2.boxPoints(box)
        box = np.array(box, dtype="int")

        # order the points in the contour such that they appear in top-left, top-right, bottom-right, and bottom-left
        # order, then draw the outline of the rotated bounding box
        box = perspective.order_points(box)
        cv2.drawContours(orig, [box.astype("int")], -1, (0, 255, 0), 2)
        # loop over the original points and draw them
        for (x, y) in box:
            cv2.circle(orig, (int(x), int(y)), 3, (0, 0, 255), -1)

        # unpack the ordered bounding box, then compute the midpoint
        # between the top-left and top-right coordinates, followed by
        # the midpoint between bottom-left and bottom-right coordinates
        (tl, tr, br, bl) = box
        (tltrX, tltrY) = midpoint(tl, tr)
        (blbrX, blbrY) = midpoint(bl, br)
        # compute the midpoint between the top-left and top-right points,
        # followed by the midpoint between the top-righ and bottom-right
        (tlblX, tlblY) = midpoint(tl, bl)
        (trbrX, trbrY) = midpoint(tr, br)
        # draw the midpoints on the image
        cv2.circle(orig, (int(tltrX), int(tltrY)), 3, (255, 0, 0), -1)
        cv2.circle(orig, (int(blbrX), int(blbrY)), 3, (255, 0, 0), -1)
        cv2.circle(orig, (int(tlblX), int(tlblY)), 3, (255, 0, 0), -1)
        cv2.circle(orig, (int(trbrX), int(trbrY)), 3, (255, 0, 0), -1)
        # draw lines between the midpoints
        cv2.line(orig, (int(tltrX), int(tltrY)), (int(blbrX), int(blbrY)), (255, 0, 255), 1)
        cv2.line(orig, (int(tlblX), int(tlblY)), (int(trbrX), int(trbrY)), (255, 0, 255), 1)

        # compute the Euclidean distance between the midpoints
        dA = dist.euclidean((tltrX, tltrY), (blbrX, blbrY))
        dB = dist.euclidean((tlblX, tlblY), (trbrX, trbrY))
        # if the pixels per metric has not been initialized, then
        # compute it as the ratio of pixels to supplied metric
        # (in this case, inches)
        if pixelsPerMetric is None:
            pixelsPerMetric = dB / args["width"]

        dimA = dA / pixelsPerMetric
        dimB = dB / pixelsPerMetric

        sizes.append((dimA,dimB))

        # draw the object sizes on the image
        cv2.putText(orig, "{:.1f}in".format(dimA), (int(tltrX - 15), int(tltrY - 10)), cv2.FONT_HERSHEY_SIMPLEX,
                   0.4, (255, 255, 255), 1)
        cv2.putText(orig, "{:.1f}in".format(dimB), (int(trbrX + 10), int(trbrY)), cv2.FONT_HERSHEY_SIMPLEX,
                   0.4, (255, 255, 255), 1)

    show_cv("size", orig)
    return sizes

def find_contours(img):
    # change image to grayscale
    imgray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # perform edge detection, then perform a dilation + erosion to
    # close gaps in between object edges
    edges = cv2.Canny(imgray, 10, 100)
    edges = cv2.dilate(edges, None, iterations=1)
    edges = cv2.erode(edges, None, iterations=1)

    # find contours in the edge map
    contours, hierarchy = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    cv2.drawContours(img, contours, -1, (250, 0, 0), 1)
    # show_cv('contours', img)
    return contours, img

def get_extrema(contours,img):
    c = max(contours, key=cv2.contourArea)

    extLeft = tuple(c[c[:, :, 0].argmin()][0])
    extRight = tuple(c[c[:, :, 0].argmax()][0])
    extTop = tuple(c[c[:, :, 1].argmin()][0])
    extBot = tuple(c[c[:, :, 1].argmax()][0])

    cent = find_center(cnt, img)
    topMid = cent[0][0], extTop[1]
    botMid = cent[0][0], extBot[1]
    leftMid = extLeft[0], cent[0][1]
    rightMid = extRight[0], cent[0][1]

    cv2.drawContours(img, [c], -1, (0, 255, 255), 2)

    cv2.circle(img, topMid, 8, (255, 0, 255), -1)
    cv2.circle(img, botMid, 8, (255, 0, 255), -1)
    cv2.circle(img, leftMid, 8, (255, 255, 0), -1)
    cv2.circle(img, rightMid, 8, (255, 255, 0), -1)

    show_cv("extrema", img)

if __name__ == "__main__":
    i = 'shapes_hex.jpg'
    img = cv2.imread(i)
    cnt, im = find_contours(img)
    # find_center(cnt, im)
    get_extrema(cnt,im)
    # find_size(cnt, im)
