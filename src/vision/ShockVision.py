import libjevois as jevois
import cv2
import numpy as np

class ShockVision:

    def getCenterX(goodContour):
        return goodContour[1]

    def process(self, inframe, outframe):
        img = inframe.getCvBGR()
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lowerMask = cv2.inRange(hsv, np.array([60, 120, 240]), np.array([100, 255, 255]))
        upperMask = cv2.inRange(hsv, np.array([60, 120, 240]), np.array([100, 255, 255]))
        mask = cv2.bitwise_or(lowerMask, upperMask)
        colorFiltered = cv2.bitwise_and(img, img, mask=upperMask)
        img=colorFiltered
        contours, hierarchy = cv2.findContours(cv2.split(colorFiltered)[2], cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_TC89_KCOS)
        goodContours = list()
        for contour in contours:
            currentContour = cv2.convexHull(contour, False)
            perimeter = cv2.arcLength(currentContour,True)
            rect = cv2.minAreaRect(currentContour)
            center = rect[0]
            angle = rect[2]
            width,height = rect[1]
            area = width * height
            apRatio=0
            if (area > 0 and perimeter > 0):
                apRatio = area/perimeter**2
            #jevois.LINFO("per: " + str(perimeter))
            #jevois.LINFO("area: " + str(area))
            #jevois.LINFO(str(apRatio))
            if width==0 or height==0:
                continue
            centerX,centerY = rect[0]
            ratio = width/height
            if (ratio > 0.2 and ratio < 0.8) and (angle > -30 and angle < 10 and apRatio > 0.045 and apRatio < 0.08):
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                isLeft = False
                img = cv2.drawContours(img, [box], 0, (0,0,255), 3)
                goodContours.append([contour, centerX, centerY, area, isLeft])
            if (ratio > 1 and ratio < 4) and (angle > -90 and angle < -50 and apRatio > 0.045 and apRatio <0.08):
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                isLeft = True
                img = cv2.drawContours(img, [box], 0, (255,0,0), 3)
                goodContours.append([contour, centerX, centerY, area, isLeft])
                
        goodContours.sort(key=ShockVision.getCenterX)
        pair = []
        if (len(goodContours)) > 0:
            largestRect = goodContours[0]
            indexOfLargest = 0
            for i in range(0,len(goodContours)):
                if goodContours[i][3] > largestRect[3]:
                    largestRect = goodContours[i]
                    indexOfLargest = i
            pair = self.pairRect(goodContours, largestRect)
            img = cv2.drawContours(img, pair[0], 0, (150,100,150), 3)
            if len(pair) == 2:
                img = cv2.drawContours(img, pair[1], 0, (150,100,150), 3)
        if len(pair) == 1 or len(pair) == 2:
            avgX = pair[0][1] + pair[1][1] if len(pair) == 2 else pair[0][1]
            avgY = pair[0][2] + pair[1][2] if len(pair) == 2 else pair[0][2]
            avgX /= len(pair)
            avgY /= len(pair)
            if pair[0][4] == True:
                TargetAngle = "L"
            else:
                TargetAngle = "R"
            jevois.sendSerial("&" + str(avgX) + "," + str(avgY) + "," + "B" + "," + str(area) + "," + "top" ) if len(pair) == 2 else jevois.sendSerial("&" + str(avgX) + "," + str(avgY) + "," + TargetAngle + "," + str(area) + "," + "top" )
        else:
            jevois.sendSerial("&None" + "," + "top")
            
        outframe.sendCv(img)
        
    def processNoUSB(self, inframe):
        img = inframe.getCvBGR()
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lowerMask = cv2.inRange(hsv, np.array([60, 120, 240]), np.array([100, 255, 255]))
        upperMask = cv2.inRange(hsv, np.array([60, 120, 240]), np.array([100, 255, 255]))
        mask = cv2.bitwise_or(lowerMask, upperMask)
        colorFiltered = cv2.bitwise_and(img, img, mask=upperMask)
        img=colorFiltered
        contours, hierarchy = cv2.findContours(cv2.split(colorFiltered)[2], cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_TC89_KCOS)
        goodContours = list()
        for contour in contours:
            currentContour = cv2.convexHull(contour, False)
            perimeter = cv2.arcLength(currentContour,True)
            rect = cv2.minAreaRect(currentContour)
            center = rect[0]
            angle = rect[2]
            width,height = rect[1]
            area = width * height
            apRatio=0
            if (area > 0 and perimeter > 0):
                apRatio = area/perimeter**2
            #jevois.LINFO("per: " + str(perimeter))
            #jevois.LINFO("area: " + str(area))
            #jevois.LINFO(str(apRatio))
            if width==0 or height==0:
                continue
            centerX,centerY = rect[0]
            ratio = width/height
            if (ratio > 0.2 and ratio < 0.8) and (angle > -30 and angle < 10 and apRatio > 0.045 and apRatio < 0.08):
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                isLeft = False
                goodContours.append([contour, centerX, centerY, area, isLeft])
            if (ratio > 1 and ratio < 4) and (angle > -90 and angle < -50 and apRatio > 0.045 and apRatio <0.08):
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                isLeft = True
                goodContours.append([contour, centerX, centerY, area, isLeft])
                
        goodContours.sort(key=ShockVision.getCenterX)
        pair = []
        if (len(goodContours)) > 0:
            largestRect = goodContours[0]
            indexOfLargest = 0
            for i in range(0,len(goodContours)):
                if goodContours[i][3] > largestRect[3]:
                    largestRect = goodContours[i]
                    indexOfLargest = i
            pair = self.pairRect(goodContours, largestRect)
                    
        if len(pair) == 1 or len(pair) == 2:
            avgX = pair[0][1] + pair[1][1] if len(pair) == 2 else pair[0][1]
            avgY = pair[0][2] + pair[1][2] if len(pair) == 2 else pair[0][2]
            avgX /= len(pair)
            avgY /= len(pair)
            if pair[0][4] == True:
                TargetAngle = "L"
            else:
                TargetAngle = "R"
            jevois.sendSerial("&" + str(int(round(avgX))) + "," + str(int(round(avgY))) + "," + "B" + "," + str(int(round(area))) + "," + "top" ) if len(pair) == 2 else jevois.sendSerial("&" + str(int(round(avgX))) + "," + str(int(round(avgY))) + "," + TargetAngle + "," + str(int(round(area))) + "," + "top")
        else:
            jevois.sendSerial("&None" + "," + "top")
    
    def pairRect(self, goodContours, largest):
        closestOpposite = None
        for contour in goodContours:
            if not contour[4] == largest[4]:
                if largest[4]:
                    if closestOpposite is None:
                        if contour[1] > largest[1]:
                            closestOpposite = contour
                    else:
                        if contour[1] < closestOpposite[1] and contour[1] > largest[1]:
                            closestOpposite = contour
                else:
                    if closestOpposite is None:
                        if contour[1] < largest[1]:
                            closestOpposite = contour
                    else:
                        if contour[1] > closestOpposite[1] and contour[1] < largest[1]:
                            closestOpposite = contour
        
        if closestOpposite is None:
            return list([largest])
        return list([largest, closestOpposite])