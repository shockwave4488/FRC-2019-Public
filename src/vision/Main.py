import libjevois as jevois
import cv2
import numpy as np

class Tests:

    def process(self, inframe, outframe):
        img = inframe.getCvBGR()
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lowerMask = cv2.inRange(hsv, np.array([60, 120, 240]), np.array([100, 255, 255]))
        upperMask = cv2.inRange(hsv, np.array([60, 120, 240]), np.array([100, 255, 255]))
        mask = cv2.bitwise_or(lowerMask, upperMask)
        colorFiltered = cv2.bitwise_and(img, img, mask=upperMask)
        
        image, contours, hierarchy = cv2.findContours(cv2.split(colorFiltered)[2], cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_TC89_KCOS)
        goodContours = []
        for contour in contours:
            currentContour = cv2.convexHull(contour, False)
            poly = cv2.approxPolyDP(currentContour, 3.5, True)
            if len(poly) < 4 or len(poly) > 8:
                continue
            lowX = poly[0][0][0]
            highX = poly[0][0][0]
            lowY = poly[0][0][1]
            highY = poly[0][0][1]
            for point in poly:
                if point[0][0] < lowX:
                    lowX = point[0][0]
                if point[0][0] > highX:
                    highX = point[0][0]
                if point[0][1] < lowY:
                    lowY = point[0][1]
                if point[0][1] > highY:
                    highY = point[0][1]
            width = highX - lowX
            height = highY - lowY
            centerX = (lowX + highX) / 2
            centerY = (lowY + highY) / 2
            ratio = width/height
            if ratio > 1 and ratio < 5:
                img = cv2.drawContours(img, [currentContour], 0, (0,0,255), 3)
                goodContours.append([contour, centerX, centerY])
                
        if len(goodContours) == 1 or len(goodContours) == 2:
            avgX = 0
            avgY = 0
            for contour in goodContours:
                avgX += contour[1]
                avgY += contour[2]
            avgX /= len(goodContours)
            avgY /= len(goodContours)
            jevois.sendSerial("&" + str(160 - avgX) + "," + str(120 - avgY))
        else:
            jevois.sendSerial("&None")
            
        outframe.sendCv(img)
        
    def processNoUSB(self, inframe):
        img = inframe.getCvBGR()
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lowerMask = cv2.inRange(hsv, np.array([60, 120, 240]), np.array([100, 255, 255]))
        upperMask = cv2.inRange(hsv, np.array([60, 120, 240]), np.array([100, 255, 255]))
        mask = cv2.bitwise_or(lowerMask, upperMask)
        colorFiltered = cv2.bitwise_and(img, img, mask=upperMask)
        
        image, contours, hierarchy = cv2.findContours(cv2.split(colorFiltered)[2], cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_TC89_KCOS)
        goodContours = []
        for contour in contours:
            currentContour = cv2.convexHull(contour, False)
            poly = cv2.approxPolyDP(currentContour, 3.5, True)
            if len(poly) < 4 or len(poly) > 8:
                continue
            lowX = poly[0][0][0]
            highX = poly[0][0][0]
            lowY = poly[0][0][1]
            highY = poly[0][0][1]
            for point in poly:
                if point[0][0] < lowX:
                    lowX = point[0][0]
                if point[0][0] > highX:
                    highX = point[0][0]
                if point[0][1] < lowY:
                    lowY = point[0][1]
                if point[0][1] > highY:
                    highY = point[0][1]
            width = highX - lowX
            height = highY - lowY
            centerX = (lowX + highX) / 2
            centerY = (lowY + highY) / 2
            ratio = width/height
            if ratio > 1 and ratio < 5:
                img = cv2.drawContours(img, [currentContour], 0, (0,0,255), 3)
                goodContours.append([contour, centerX, centerY])
                
        if len(goodContours) == 1 or len(goodContours) == 2:
            avgX = 0
            avgY = 0
            for contour in goodContours:
                avgX += contour[1]
                avgY += contour[2]
            avgX /= len(goodContours)
            avgY /= len(goodContours)
            jevois.sendSerial("&" + str(160 - avgX) + "," + str(120 - avgY))
        else:
            jevois.sendSerial("&None")