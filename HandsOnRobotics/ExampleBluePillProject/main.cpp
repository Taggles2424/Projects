
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>

using namespace std;
using namespace cv;

int main() {
  cout << "Hello World!" << endl;

  // Try loading the image
  Mat image = imread("../test_image.jpg");

  // Check for failure
  if (image.empty()) {
    cout << "ERROR: Could not open or find the image at '../test_image.jpg'"
         << endl;
    cout << "Please ensure the executable is run from the 'build' folder."
         << endl;
    cin.get(); // Wait for key press
    return -1;
  }

  // Success diagnostic log
  cout << "SUCCESS: Image loaded successfully!" << endl;
  cout << "Image Dimensions: " << image.cols << "x" << image.rows << " px"
       << endl;
  cout << "Image Channels: " << image.channels() << endl;

  String windowName = "AAU"; // Name of the window

  // Create window with normal flag to allow resizing
  namedWindow(windowName, WINDOW_NORMAL);
  resizeWindow(windowName, image.cols, image.rows);

  cout << "Displaying window..." << endl;
  imshow(windowName, image); // Show our image inside the created window.

  // Warm-up loop to pump the GTK/GDK GUI event queue (fixes blank screen in
  // WSLg/X11)
  for (int i = 0; i < 10; i++) {
    waitKey(30);
  }

  cout << "Press any key in the image window to close it..." << endl;
  waitKey(0); // Wait for any keystroke in the window

  return 0;
}
