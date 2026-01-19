import cv2
import numpy as np

def nothing(x):
    pass

# Create window and trackbars
cv2.namedWindow("HSV Tuner")
cv2.createTrackbar("H min", "HSV Tuner", 35, 180, nothing)
cv2.createTrackbar("H max", "HSV Tuner", 85, 180, nothing)
cv2.createTrackbar("S min", "HSV Tuner", 80, 255, nothing)
cv2.createTrackbar("S max", "HSV Tuner", 255, 255, nothing)
cv2.createTrackbar("V min", "HSV Tuner", 80, 255, nothing)
cv2.createTrackbar("V max", "HSV Tuner", 255, 255, nothing)

cap = cv2.VideoCapture(1)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Read slider values
    h_min = cv2.getTrackbarPos("H min", "HSV Tuner")
    h_max = cv2.getTrackbarPos("H max", "HSV Tuner")
    s_min = cv2.getTrackbarPos("S min", "HSV Tuner")
    s_max = cv2.getTrackbarPos("S max", "HSV Tuner")
    v_min = cv2.getTrackbarPos("V min", "HSV Tuner")
    v_max = cv2.getTrackbarPos("V max", "HSV Tuner")

    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])

    # Mask + result
    mask = cv2.inRange(hsv, lower, upper)
    result = cv2.bitwise_and(frame, frame, mask=mask)

    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)

        if area > 200:
            # Draw contour outline
            cv2.drawContours(frame, [largest], -1, (0, 255, 255), 2)

            # Compute centroid
            M = cv2.moments(largest)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                cv2.circle(frame, (cx, cy), 8, (0, 255, 0), -1)
                cv2.putText(frame, f"({cx},{cy})", (cx + 10, cy - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # Show area
            cv2.putText(frame, f"Area: {int(area)}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            # =========================================================
            # AUTOMATIC THRESHOLD SUGGESTION
            # =========================================================
            mask_roi = np.zeros_like(mask)
            cv2.drawContours(mask_roi, [largest], -1, 255, -1)

            # Extract HSV pixels inside contour
            blob_pixels = hsv[mask_roi == 255]

            if len(blob_pixels) > 0:
                h_vals = blob_pixels[:, 0]
                s_vals = blob_pixels[:, 1]
                v_vals = blob_pixels[:, 2]

                suggested_lower = np.array([
                    max(0, int(np.percentile(h_vals, 5))),
                    max(0, int(np.percentile(s_vals, 5))),
                    max(0, int(np.percentile(v_vals, 5)))
                ])

                suggested_upper = np.array([
                    min(180, int(np.percentile(h_vals, 95))),
                    min(255, int(np.percentile(s_vals, 95))),
                    min(255, int(np.percentile(v_vals, 95)))
                ])

                # Display suggestion on screen
                cv2.putText(frame,
                            f"Suggested H: {suggested_lower[0]} - {suggested_upper[0]}",
                            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (0, 200, 255), 2)
                cv2.putText(frame,
                            f"Suggested S: {suggested_lower[1]} - {suggested_upper[1]}",
                            (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (0, 200, 255), 2)
                cv2.putText(frame,
                            f"Suggested V: {suggested_lower[2]} - {suggested_upper[2]}",
                            (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (0, 200, 255), 2)

                # Print to terminal too
                print("Suggested HSV ranges:")
                print("Lower:", suggested_lower)
                print("Upper:", suggested_upper)
                print()

    combined = np.hstack((frame, result))
    cv2.imshow("HSV Tuner", combined)

    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
