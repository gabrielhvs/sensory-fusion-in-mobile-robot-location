import numpy as np
import cv2 as cv
import glob

# Dimensões do Tabuleiro de Xadrez
cbcol = 7
cbrow = 10
cbw = 15

# Critério
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, cbw, 0.001)


# preparar os pontos do objeto, como (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((cbrow * cbcol, 3), np.float32)
objp[:, :2] = np.mgrid[0:cbcol, 0:cbrow].T.reshape(-1, 2)
# Vetores para armazenar os pontos de objeto e pontos de imagem de todas as imagens.
objpoints = [] # ponto 3d no espaço do mundo real
imgpoints = [] # ponto 2d no plano da imagem.
images = glob.glob('/home/robottraining/Ararajuba/src/realsense/calibração/*.png')
print(images)
i=0
for fname in images:
    print(fname)
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # Encontra os cantos do tabuleiro de xadrez
    ret, corners = cv.findChessboardCorners(gray, (cbcol,cbrow), None)
    # Se encontrado, adiciona os pontos de objeto e pontos de imagem (após refiná-los)
    print(ret)
    if ret == True:
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners)
        # Desenha e mostra os cantos
        cv.drawChessboardCorners(img, (cbcol, cbrow), corners2, ret)
        cv.imwrite('/home/robottraining/Ararajuba/src/realsense/calibração/images' + str(i) + '.png', img)
        cv.imshow('img', img)
        cv.waitKey(500)
        i += 1
cv.destroyAllWindows()

ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
print("mtx" , mtx)
print("dist" , dist)
print("rvecs" , rvecs)
print("tvecs" , tvecs)