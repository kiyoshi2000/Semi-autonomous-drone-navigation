import cv2
import numpy as np
#import opencv_video_tools as ovt
import sys


def from_lsd(lines_lsd):
    """
    lines_lsd : the weird output of cv line segment detector.
    returns : [[x1, y1, x2, y2, a, b, c, L],
               [...                       ],
               ...
              ]

              where 
                - [(x1, y1), (x2, y2)] is the segment (in image coordinates).
                - ax + by + c = 0 is the line equation
                - (.
                - |(a, b)| = 1, b >= 0
                - L is the length of the segment.
    """
    if lines_lsd is None :
        return np.array([]).reshape((0,8))

    nlines = lines_lsd.shape[0]
    segments = None

    if nlines == 0 :
        return np.array([]).reshape((0,8))

    if nlines == 1 :
        segments = lines_lsd.squeeze().reshape((1,4))
    else :
        segments = lines_lsd.squeeze()

    x1 = segments[:, 0]
    y1 = segments[:, 1]
    x2 = segments[:, 2]
    y2 = segments[:, 3]
    b = x1 - x2
    a = y2 - y1
    c = -(a * x1 + b * y1)

    # Ensure that the normal vectors are always
    # pointing toward positive y
    where_b_is_neg = b < 0
    c[where_b_is_neg] *= -1.0
    b[where_b_is_neg] *= -1.0
    a[where_b_is_neg] *= -1.0

    # The length of the segment
    norm = np.sqrt(a**2 + b**2)
    return np.stack([x1, y1, x2, y2, a/norm, b/norm, c/norm, norm], axis=1)

def draw_segments(img, segments, color, thickness):
    '''
        img : cv2 image
        segments : np.array(num_lines, 8)
        color  : 3 element tuple
        thickness : int
    '''

    for s in segments:
        cv2.line(img, (s[0], s[1]), (s[2], s[3]), color, thickness)


def draw_lines(img, segments, color, thickness):
    '''
        img : cv2 image
        segments : np.array(num_lines, 8)
        color  : 3 element tuple
        thickness : int
    '''

    for s in segments:
        a, b, c = s[4:7]
        # If the line is horizontal
        if a == 0:
            cv2.line(img, (0, -int(c/b)), (img.shape[1]-1, -int((c+a*(img.shape[1]-1))/b)), color, thickness)
        # If the line is vertical
        elif b == 0:
            cv2.line(img, (-int(c/a), 0), (-int(c/a), img.shape[0] - 1), color, thickness)
        else:
            # We look for the intersection point with the horizontal axis y=0 and y=height-1
            y = 0
            x = -c/a   #-(c - b*y)/a
            if x < 0:
                x = 0
                y = -(c + a * x) / b
            elif x >= img.shape[1]:
                x = img.shape[1] - 1
                y = -(c + a * x) / b
            first_point = (int(x), int(y))

            y = img.shape[0] - 1
            x = -(c+b*y)/a
            if x < 0:
                x = 0
                y = -(c + a * x) / b
            elif x >= img.shape[1]:
                x = img.shape[1] - 1
                y = -(c + a * x) / b
            second_point = (int(x), int(y))

            cv2.line(img, first_point, second_point, color, thickness)
            
            print()

def intersections(segments):

    intersections = []
    for i, si in enumerate(segments):
        for sj in segments[i+1:]:
            cross_product = np.cross(si[4:6], sj[4:6]) # [a1,b1] ^ [a2, b2]
            if cross_product != 0:
                coeff = 1.0 / cross_product

                intersections.append([coeff * np.cross(si[5:7]   , sj[5:7]), # [b1, c1] ^ [b2, c2]
                                      coeff * np.cross(sj[[4, 6]], si[[4, 6]])]) # -[a1, c1] ^ [a2, c2]
                
    return np.array(intersections)

def distancia(p1, p2):
    return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def encontrar_ponto_mais_proximo(pontos, ponto_referencia):
    if not pontos.any():
        return None

    distancias = list(map(lambda ponto: distancia(ponto, ponto_referencia), pontos))

    # Encontre o índice do ponto mais próximo
    indice_ponto_mais_proximo = distancias.index(min(distancias))
    
    # Use o índice encontrado para obter o ponto de treinamento mais próximo
    ponto_mais_proximo = pontos[indice_ponto_mais_proximo]

    return ponto_mais_proximo

def Vanishing_point(img,pontos,vpa):

    if not pontos.any():
        return vpa



    ponto_mais_proximo = encontrar_ponto_mais_proximo(pontos, vpa)

    x,y = ponto_mais_proximo
    x=int(x)
    y=int(y)

    return (x,y)
        

def filterlines(segments):
   
    FinalLines = []

    for l1 in segments:
        [x1, y1, x2, y2, a, b, c, L] = l1
        n1 = (a,b)
        if abs(a) > 0.1 and abs(b)> 0.1:
            for l2 in segments:
                [X1, Y1, X2, Y2, A, B, C, l] = l2
                n2 = (A,B)
                if abs(A) > 0.1 and abs(B)> 0.1:
                    print(n1)
                    if (abs(a/abs(a) - A/abs(A))!=0 and abs(b/abs(b) - B/abs(B))==0) or (abs(a/abs(a) - A/abs(A))==0 and abs(b/abs(b) - B/abs(B))!=0):
                        FinalLines.append(l1)

    return FinalLines

def draw_intersections(img,pontos):
    for p in pontos:
        (x,y) = p
        x = int(x)
        y = int(y)
        cv2.circle(img,(x,y),2,(255,0,0),2)



def horizontal_misplacement(ponto, val_min, val_max):

    (x,y) = ponto
      
    n_v= (x-val_min)/(val_max-val_min)

    value1= 2*n_v -1

    #print(value1)

    return value1 




def unbalance_two_angles(img,ponto):

    (x,y) = ponto

    tx = img.shape[1]

    alfa1=np.arctan(y/x)
    alfa2=np.arctan(y/(tx-x))

    value2=(alfa1-alfa2)/(np.pi/2)
    value2=2*value2-1
    
    return value2
