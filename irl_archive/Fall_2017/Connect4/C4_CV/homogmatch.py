'''
DOES NOT WORK BY ITSELF
Tried to use in c4imageproc.py
but was very sad
moved here 12/6/17
'''
def homog_match(self, trainimage, frame):
      MIN_MATCH_COUNT = 10
      sift = cv2.xfeatures2d.SIFT_create()
      kp1, des1 = sift.detectAndCompute(trainimage,None)
      kp2, des2 = sift.detectAndCompute(frame,None)

      FLANN_INDEX_KDTREE = 0
      index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees=5)
      search_params = dict(checks = 50)

      flann = cv2.FlannBasedMatcher(index_params, search_params)
      matches = flann.knnMatch(des1, des2, k=2)

      good = []
      for m, n in matches:
          if m.distance < 1.5*n.distance:
              good.append(m)

      if len(good)>MIN_MATCH_COUNT:
          print "Found enough matches"
          src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
          dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

          M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
          matchesMask = mask.ravel().tolist()

          h,w = trainimage.shape
          pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
          dst = cv2.perspectiveTransform(pts,M)

          #cv2.polylines(frame,[np.int32(dst)],True,150,3, cv2.LINE_AA)

          ''''visualize matches'''
          draw_params = dict(matchColor = (0, 255,0), singlePointColor = None, matchesMask = matchesMask, flags = 2)
          img3 = cv2.drawMatches(trainimage, kp1, frame, kp2, good, None, **draw_params)
          plt.imshow(img3, 'gray'), plt.show()
          return dst

      else:
          print "Not enough matches are found - %d/%d" % (len(good),MIN_MATCH_COUNT)
          matchesMask = None
