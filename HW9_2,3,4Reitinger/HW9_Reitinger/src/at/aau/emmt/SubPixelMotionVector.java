package at.aau.emmt;

import java.awt.Color;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class SubPixelMotionVector {

    public static final int[] filter = { 1, -5, 20, 20, -5, 1 };

    public static void main(String args[]) {

        /**
         * Use this code for testing.
         */
        int[][] matrix = {
                { 10, 14, 7, 5 },
                { 16, 12, 13, 19 },
                { 14, 12, 1, 5 },
                { 8, 2, 3, 3 }
        };

        int[][] test = interpolateFrame(matrix);

        /**
         * Use this code if you want to see the result as image. You can also use the
         * test.png image for testing.
         */
        int[][] r_frame = getLumincanceChannel(
                "c:/Users/maxir/OneDrive/Desktop/Antigravity/HW9_2,3,4Reitinger/HW9_Reitinger/image-0096.png");
        int[][] t_frame = getLumincanceChannel(
                "c:/Users/maxir/OneDrive/Desktop/Antigravity/HW9_2,3,4Reitinger/HW9_Reitinger/image-0097.png");

        int[][] r_frame_sub = interpolateFrame(r_frame);

        // To print the interpolated image
        BufferedImage img = new BufferedImage(r_frame_sub[0].length,
                r_frame_sub.length, BufferedImage.TYPE_INT_RGB);
        for (int col = 0; col < r_frame_sub[0].length; col++) {
            for (int row = 0; row < r_frame_sub.length; row++) {
                Color c = new Color(r_frame_sub[row][col], r_frame_sub[row][col],
                        r_frame_sub[row][col]);
                img.setRGB(col, row, c.getRGB());
            }
        }
        try {
            ImageIO.write(img, "PNG", new File("test.png")); // TODO adapt to your needs
        } catch (IOException e) {
            System.out.println("Could not save Image\n");
            System.exit(-1);
        }

        // TODO Activate for Motion Vector Calculation
        searchMotionVector(r_frame, t_frame, 8, 32, 688, 688, 1);
        searchMotionVector(r_frame_sub, t_frame, 8, 32, 688, 688, 4);
        System.out.println();

        searchMotionVector(r_frame, t_frame, 8, 32, 128, 128, 1);
        searchMotionVector(r_frame_sub, t_frame, 8, 32, 128, 128, 4);
        System.out.println();

        searchMotionVector(r_frame, t_frame, 8, 32, 328, 328, 1);
        searchMotionVector(r_frame_sub, t_frame, 8, 32, 328, 328, 4);
        System.out.println();

        searchMotionVector(r_frame, t_frame, 8, 32, 400, 400, 1);
        searchMotionVector(r_frame_sub, t_frame, 8, 32, 400, 400, 4);
        System.out.println();

        searchMotionVector(r_frame, t_frame, 8, 32, 512, 512, 1);
        searchMotionVector(r_frame_sub, t_frame, 8, 32, 512, 512, 4);
    }

    /**
     * Interpolates a frame with quarter subpixel precision
     *
     * @param r_frame
     * @return
     */
    private static int[][] interpolateFrame(int[][] r_frame) {

        if (r_frame.length < 10) {
            System.out.println("Unchanged Table (Ex 1 - Full Pixels):");
            System.out.println(matrixToString(r_frame));
        }

        // TODO Implement for HP1
        // create a larger copy of the image and copy existing pixels on the right
        // places
        int h = r_frame.length;
        int w = r_frame[0].length;
        int[][] new_frame = new int[(h - 1) * 4 + 1][(w - 1) * 4 + 1];

        for (int i = 0; i < h; i++) {
            for (int j = 0; j < w; j++) {
                new_frame[i * 4][j * 4] = r_frame[i][j];
            }
        }

        if (r_frame.length < 10) {
            System.out.println("Table 2 (Extended Matrix - Full Pixels):");
            System.out.println(matrixToString(new_frame));
        }

        int[] tmp = { 0, 0, 0, 0, 0, 0 }; // E F G H I J or A C G M R T

        // TODO Implement for HP2
        // calculate the horizontal half-pixels and place them correctly
        // you can use the existing filter for this task
        for (int i = 0; i < h; i++) {
            for (int j = 0; j < w - 1; j++) {
                // Horizontal half-pixel between j and j+1 (at index 4*j + 2)
                // Filter taps: j-2, j-1, j, j+1, j+2, j+3
                for (int k = 0; k < 6; k++) {
                    int pos = j - 2 + k;
                    // Boundary handling: always duplicate the last pixel value
                    if (pos < 0)
                        pos = 0;
                    if (pos >= w)
                        pos = w - 1;
                    tmp[k] = r_frame[i][pos];
                }
                new_frame[i * 4][j * 4 + 2] = useFilter(tmp);
            }
        }

        if (r_frame.length < 10) {
            System.out.println("Intermediate Table (Ex 2 - Horizontal Half-pel):");
            System.out.println(matrixToString(new_frame));
        }

        // TODO Implement for HP3
        // calculate the vertical half-pixels and place them correctly
        // you can use the existing filter for this task
        for (int i = 0; i < h - 1; i++) {
            for (int j = 0; j < w; j++) {
                // Vertical half-pixel between i and i+1 (at index 4*i + 2)
                for (int k = 0; k < 6; k++) {
                    int pos = i - 2 + k;
                    if (pos < 0)
                        pos = 0;
                    if (pos >= h)
                        pos = h - 1;
                    tmp[k] = r_frame[pos][j];
                }
                new_frame[i * 4 + 2][j * 4] = useFilter(tmp);
            }
        }

        // TODO Implement for HP4
        // interpolate the remaining ("diagonal") half-pixels via vertical OR horizontal
        // method.
        // you can use the existing filter for this task
        // We use Vertical interpolation on horizontal half-pel results as instructed.
        for (int i = 0; i < h - 1; i++) {
            for (int j = 0; j < w - 1; j++) {
                // Diagonal half-pixel at [4*i + 2][4*j + 2]
                for (int k = 0; k < 6; k++) {
                    int pos = i - 2 + k;
                    if (pos < 0)
                        pos = 0;
                    if (pos >= h)
                        pos = h - 1;
                    tmp[k] = new_frame[pos * 4][j * 4 + 2];
                }
                new_frame[i * 4 + 2][j * 4 + 2] = useFilter(tmp);
            }
        }

        if (r_frame.length < 10) {
            System.out.println("Table 3 (h-pel Interpolation):");
            System.out.println(matrixToString(new_frame));
        }

        // TODO Implement for QP1, QP2, QP3
        // interpolate the horizontal, vertical and diagonal quarter pixels
        for (int i = 0; i < new_frame.length; i++) {
            for (int j = 0; j < new_frame[0].length; j++) {
                if (i % 2 == 0 && j % 2 != 0) {
                    // QP1: Horizontal quarter-pel or between half-pels
                    new_frame[i][j] = (int) Math.round((new_frame[i][j - 1] + new_frame[i][j + 1]) / 2.0);
                } else if (i % 2 != 0 && j % 2 == 0) {
                    // QP2: Vertical quarter-pel or between half-pels
                    new_frame[i][j] = (int) Math.round((new_frame[i - 1][j] + new_frame[i + 1][j]) / 2.0);
                }
            }
        }

        for (int i = 1; i < new_frame.length; i += 2) {
            for (int j = 1; j < new_frame[0].length; j += 2) {
                // QP3: Diagonal quarter-pel (both odd coordinates)
                // Average of the closest horizontal half-pel and vertical half-pel
                int r_hpel = (i % 4 == 1) ? i - 1 : i + 1;
                int c_hpel = (j % 4 == 1) ? j + 1 : j - 1;
                int r_vpel = (i % 4 == 1) ? i + 1 : i - 1;
                int c_vpel = (j % 4 == 1) ? j - 1 : j + 1;

                // Boundary check for indices
                if (r_hpel >= new_frame.length)
                    r_hpel = new_frame.length - 1;
                if (c_hpel >= new_frame[0].length)
                    c_hpel = new_frame[0].length - 1;
                if (r_vpel >= new_frame.length)
                    r_vpel = new_frame.length - 1;
                if (c_vpel >= new_frame[0].length)
                    c_vpel = new_frame[0].length - 1;

                new_frame[i][j] = (int) Math.round((new_frame[r_hpel][c_hpel] + new_frame[r_vpel][c_vpel]) / 2.0);
            }
        }

        if (r_frame.length < 10) {
            System.out.println("Table 4 (q-pel Interpolation):");
            System.out.println(matrixToString(new_frame));
        }

        return new_frame;
    }

    /**
     * @param rFrame            the reference frame of a video (actually just 1
     *                          channel, e.g. Y')
     * @param tFrame            the target frame of a video (actually just 1
     *                          channel, e.g. Y')
     * @param blocksize         the blocksize of the macroblocks (should be 8x8 or
     *                          16x16)
     * @param searchWindow      the searchwindow size = (2*p+1) for the algorithm
     *                          (the bigger the better the result, but the worse the
     *                          performance)
     * @param rowIndex          the y-coordinate of first (top,left) element of the
     *                          macroblock in the t_frame
     * @param colIndex          the x-coordinate of the first (top,left) element of
     *                          the macroblock in the t_frame
     * @param subPixelPrecision the subpixel precision
     * @return the motion vector
     */
    private static SubPixelMotionVector searchMotionVector(int[][] rFrame, int[][] tFrame,
            int blocksize, int searchWindow,
            int rowIndex, int colIndex,
            int subPixelPrecision) {

        // TODO Include subPixelPrecision variable
        int scaledP = (searchWindow / 2) * subPixelPrecision;

        // 1. check if the specified macroblock does not exceed the frame border. if so
        // return null.

        if (rowIndex + blocksize > tFrame.length || colIndex + blocksize > tFrame[0].length) {
            return null;
        }

        // 2. extract the macroblock's data from the tFrame

        int[][] tBlock = new int[blocksize][blocksize];
        for (int i = 0; i < tBlock.length; i++) {
            for (int j = 0; j < tBlock[0].length; j++) {
                tBlock[i][j] = tFrame[i + rowIndex][j + colIndex];
            }
        }

        double minMAD = Double.MAX_VALUE;
        SubPixelMotionVector mv = new SubPixelMotionVector(0, 0);

        // 3. sequential search algorithm on rFrame to determine the motion vector
        // use MAD function as difference measure.
        for (int i = -scaledP; i <= scaledP; i++) {
            for (int j = -scaledP; j <= scaledP; j++) {
                double currentMad = MAD(tBlock, rFrame, i, j, rowIndex, colIndex, subPixelPrecision);
                if (currentMad < minMAD) {
                    minMAD = currentMad;
                    mv.setNewValues(j / (double) subPixelPrecision, i / (double) subPixelPrecision);
                }
            }
        }

        System.out.println((subPixelPrecision == 1 ? "No Interpolation" : "1/4-Subpixel")
                + " (" + colIndex * subPixelPrecision + ", " + rowIndex * subPixelPrecision + "): Min_MAD for "
                + mv.toString() + " = " + minMAD);

        return mv;
    }

    /**
     * @param tBlock            the target macroblock for which you search the MV
     * @param rFrame            the reference frame
     * @param i                 the search offset of the y coordinate
     * @param j                 the search offset of the x coordinate
     * @param rowIndex          the offset of the macroblock's y coordinate
     * @param colIndex          the offset of the macroblock's x coordinate
     * @param subPixelPrecision the subpixel precision
     * @return the MDA value
     */
    private static double MAD(int[][] tBlock, int[][] rFrame,
            int i, int j,
            int rowIndex, int colIndex,
            int subPixelPrecision) {

        // TODO Include subPixelPrecision variable
        int r_row = rowIndex * subPixelPrecision + i;
        int r_col = colIndex * subPixelPrecision + j;

        // calculate the MAD value. if you exceed the rFrame's border return
        // Double.MAX_VALUE;

        int blocksize = tBlock.length;
        if (r_row < 0 || r_row + (blocksize - 1) * subPixelPrecision >= rFrame.length ||
                r_col < 0 || r_col + (blocksize - 1) * subPixelPrecision >= rFrame[0].length) {
            return Double.MAX_VALUE;
        }

        double mad = 0;
        for (int k = 0; k < blocksize; k++) {
            for (int l = 0; l < blocksize; l++) {
                int cPx = tBlock[k][l];
                // add subpixel precision to the row and column index
                int rPx = rFrame[r_row + k * subPixelPrecision][r_col + l * subPixelPrecision];
                mad += Math.abs(cPx - rPx);

            }

        }

        return mad / (blocksize * blocksize);
    }

    /**
     * Uses a 6-tap filter on the input, tmp is assumed to has a length of 6;
     *
     * @param tmp
     * @return
     */
    private static int useFilter(int[] tmp) {

        if (tmp.length != filter.length)
            return -1;

        int val = 0;

        for (int i = 0; i < filter.length; i++)
            val += tmp[i] * filter[i];

        val = (int) Math.round(((double) val) / 32.0);

        val = Math.min(255, val);
        val = Math.max(0, val);

        return val;
    }

    private static int[][] getLumincanceChannel(String file) {
        BufferedImage image = null;
        try {
            image = ImageIO.read(new File(file));
        } catch (IOException e) {
            System.out.println("Could not load Image\n");
            System.exit(-1);
        }

        int h = image.getHeight();
        int w = image.getWidth();
        int pixel = 0;

        int[][] data = new int[h][w];
        for (int i = 0; i < h; i++) {
            for (int j = 0; j < w; j++) {
                pixel = image.getRGB(j, i);
                data[i][j] = (int) Math
                        .round(0.299 * ((pixel >> 16) & 0xFF) + 0.587 * ((pixel >> 8) & 0xFF) + 0.114 * (pixel & 0xFF));
            }
        }
        return data;
    }

    private double u, v;

    /**
     * creates a new motion vector
     *
     * @param u horizontal change
     * @param v vertical change
     */
    public SubPixelMotionVector(double u, double v) {
        this.u = u;
        this.v = v;
    }

    /**
     * updates the motion vector
     *
     * @param u horizontal change
     * @param v vertical change
     */
    public void setNewValues(double u, double v) {
        this.u = u;
        this.v = v;
    }

    public String toString() {
        return new String("MV(" + u + "," + v + ")");
    }

    public static String matrixToString(int[][] matrix) {
        String s = "";

        for (int rows = 0; rows < matrix.length; rows++)
            s += arrayToString(matrix[rows]) + "\n";

        return s;
    }

    public static String arrayToString(int[] array) {
        String out = "[ ";

        for (int i = 0; i < array.length; i++) {
            out += String.format("%02d ", array[i]);
        }

        out += "]";

        return out;
    }

}