#!/usr/bin/env python
import cv2
import numpy as np

def nothing(x):
    pass

def main():
    # Iniciar la captura de la cámara
    cap = cv2.VideoCapture(0)

    # Crear una ventana para las barras deslizantes
    cv2.namedWindow("HSV Adjustments")

    # Crear las barras deslizantes para ajustar el color
    cv2.createTrackbar("Lower Hue", "HSV Adjustments", 0, 179, nothing)
    cv2.createTrackbar("Lower Saturation", "HSV Adjustments", 0, 255, nothing)
    cv2.createTrackbar("Lower Value", "HSV Adjustments", 0, 255, nothing)
    cv2.createTrackbar("Upper Hue", "HSV Adjustments", 179, 179, nothing)
    cv2.createTrackbar("Upper Saturation", "HSV Adjustments", 255, 255, nothing)
    cv2.createTrackbar("Upper Value", "HSV Adjustments", 255, 255, nothing)

    while True:
        # Leer el cuadro actual
        ret, frame = cap.read()
        if not ret:
            break
        """
        # Convertir a espacio de color HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Obtener los valores actuales de las barras deslizantes
        l_h = cv2.getTrackbarPos("Lower Hue", "HSV Adjustments")
        l_s = cv2.getTrackbarPos("Lower Saturation", "HSV Adjustments")
        l_v = cv2.getTrackbarPos("Lower Value", "HSV Adjustments")
        u_h = cv2.getTrackbarPos("Upper Hue", "HSV Adjustments")
        u_s = cv2.getTrackbarPos("Upper Saturation", "HSV Adjustments")
        u_v = cv2.getTrackbarPos("Upper Value", "HSV Adjustments")

        # Definir el rango de color HSV para detectar colores
        lower_color = np.array([l_h, l_s, l_v])
        upper_color = np.array([u_h, u_s, u_v])

        # Crear una máscara que identifique el color especificado
        mask = cv2.inRange(hsv, lower_color, upper_color)

        # Aplicar la máscara a la imagen original
        result = cv2.bitwise_and(frame, frame, mask=mask)

        # Mostrar el resultado
        cv2.imshow("Original", frame)
        cv2.imshow("Mask", mask)
        cv2.imshow("Detected Color", result)
        """

        cv2.imshow("Original", frame)
        # Romper el bucle si se presiona la tecla 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Liberar la cámara y cerrar todas las ventanas
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
