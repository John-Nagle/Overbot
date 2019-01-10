#include "KMeans.h"
#include "Port.h"
/**
 * K-Means clustering of pixels identifying "blobs" of similar colored pixels.
 * Blobs are identified by clustering pixels into localized regions of like
 * color.  Clustering is performed on a set of feature vectors derived from the
 * input image.  A 5-dimensional feature vector is derived from each pixel according
 * to its location and colors {x, y, r, g, b}.
 * Distance in this feature vector space is defined by the Mahalanobis metric.
 * The clustering algorithm is iterative.
 * Computational complexity of each iteration is O(K*#pixels).
 * See these references for some insight into what is going on here:
 * http://www.ri.cmu.edu/pubs/pub_1641.html
 * http://www.engr.sjsu.edu/~knapp/HCIRODPR/PR_Mahal/PR_Mahal.htm
 * 
 *
 * @version $Revision: 1.1 $
 *
 * @author John Pierre
 *
 */

#include <iostream.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>





/**
 * Constructor.
 *
 * @param  k  Number of clusters
 */
KMeans::KMeans(int k) :
  m_debug(0), m_verbose(0), m_img(0),
  m_height(0), m_width(0),
  m_distribution(0), m_clusterlist(0),
  m_Ndim(0), m_assignments(0),
  m_means(0), m_K(0), m_maxiterations(MAX_ITERATIONS), m_empty(0),
  vv(0)
{
  m_K = k;
  m_Ndim = NUM_DIMENSIONS;
  allocateMeans();
  allocateCovariances();
  m_distribution = (int *)calloc(m_K, sizeof(int));
  m_clusterlist = (int *)calloc(m_K, sizeof(int));
  vv = new double[NUM_DIMENSIONS];
}

/**
 * Destructor.
 */
KMeans::~KMeans() {
  if (m_assignments) {
    free(m_assignments);
  }
  freeMeans();
  freeCovariances();
  if (m_distribution) {
      free(m_distribution);
  }
  if (m_clusterlist) {
      free(m_clusterlist);
  }
  delete vv;
}

/**
 * Set debug mode.
 *
 * @param  d  Set non-zero to turn on debug mode
 */
void KMeans::setDebug(int d) {
  m_debug = d;
}

/**
 * Set verbose mode.
 *
 * @param  v  Set non-zero to turn on verbose mode
 */
void KMeans::setVerbose(int v) {
  m_verbose = v;
}

/**
 * Set maximum number of iterations to attempt re-clustering.
 * Default is 20.
 *
 * @param  i  Max iterations
 */
void KMeans::setMaxIterations(int i) {
  m_maxiterations = i;
}

/**
 * Cluster the pixels
 *
 */
void KMeans::cluster(IplImage *img) {
    m_img = img;
    // if image size has changed we need to resize our storage
    if (img->width != m_width || img->height != m_height) {
        resize(img->width, img->height);
    }

    if (m_verbose || m_debug) {
        cout << "Initialize " << m_K << " clusters" << endl;
    }

    // initialize using random cluster assignments
    initialize();

    // initialize our measure of quality to test convergence of the algorithm
    double oldquality = 0.0;
  
    // perform iteractions of the K-means clustering algorithm
    for (int i=0; i<m_maxiterations; i++) {
        if (m_debug) {
            cout << "Begin iteration " << i+1 << endl;
        }

        // compute centroids of the clusters
        computeMeans();

        // compute covariance and inverse covariance matrixes
        computeCovariances();

        // assign pixels to their nearest cluster centroid
        double newquality = assign();

        if (m_debug || m_verbose) {
            cout << "Iteration " << i+1 << " quality=" << newquality << endl << endl;
        }

        // check stopping criterion (convergence)
        if (fabs(oldquality - newquality) <= CONVERGENCE_FACTOR * fabs(oldquality)) {
            // if quality hasn't changed much we've converged, end iterations
            break;
        }

        oldquality = newquality;
    }
}

/**
 * Get cluster assignment for a given pixel
 * @param  x  The x value of the pixel
 * @param  y  The y value of the pixel
 * @return The cluster id
 */
int KMeans::getAssignment(int x, int y) {
  return *(m_assignments + m_width*y + x);
}

/**
 * Colorize the currently loaded image using the cluster assignments and color of the
 * cooresponding mean.
 *
 * @return The colorized image.
 */
IplImage *KMeans::getColorAssignments() {

  // loop through pixels, get cluster assignment, and reset pixel colors
  for (int y=0; y<m_height; y++) {
    int *assignment_offset_ptr = m_assignments + m_width*y;
    uchar *image_offset_ptr = (uchar*)(m_img->imageData + m_img->widthStep*y);
    for (int x=0; x<m_width; x++) {

        // get the cluster assignment for the pixel at x,y
        int clusterid = *(assignment_offset_ptr + x);

        if (clusterid < 1) {
          continue;
        }

        // get the information for the cluster
        int cid = clusterid - 1;
        pixelVector pv = m_means[cid];

        // get a pointer to pixel column of pixels
        unsigned index = 3*x;

        // set the RGB values of the pixel to the RGB for the cluster mean
        (image_offset_ptr)[index] = (unsigned char)pv.b;
        (image_offset_ptr)[index+1] = (unsigned char)pv.g;
        (image_offset_ptr)[index+2] = (unsigned char)pv.r;


    }

  }

  return m_img;
}

///////////////////////////////////////////////////////////////////////////
// PRIVATE METHODS
///////////////////////////////////////////////////////////////////////////
/**
 * Internal method to initialize the clustering algorithm.
 *
 */
void KMeans::initialize() {
    
    // reset the cluster assignments
    memset(m_assignments, 0, m_Ndata*sizeof(int));

    // initialize cluster assignments
    randomInit();
}


/**
 * Internal method to re-assign documents to nearest centroid
 */
double KMeans::assign() {  
    double quality = 0.0;

    for (int y=0; y<m_height; y++) {
        int *assignment_offset_ptr = m_assignments + m_width*y;
        uchar *image_offset_ptr = (uchar*)(m_img->imageData + m_img->widthStep*y);
        for (int x=0; x<m_width; x++) {
            int *assignment_ptr = assignment_offset_ptr + x;
            int oldcluster = *assignment_ptr;

            // find nearest centroid
            double similarity = 0.0;
            int newcluster = findNearestMean(x, y,&similarity) + 1;
            quality += similarity;

            // assign this doc to the new cluster
            if (newcluster > 0) {
                *assignment_ptr = newcluster;
            }

            if (m_debug) {
                 cout << "reassign x=" << x << " y=" << y << " from " << oldcluster
                      << " to " << newcluster << endl;
            }
        }
    }

    return quality;
}

/**
 * When image is resized we need to reallocate our storage space.
 * @param  w  Width of image in pixels
 * @param  h  Height of image in pixels
 */
void KMeans::resize(int w, int h) {
    // update member variables
    m_width = w;
    m_height = h;
    m_Ndata = w*h;
    // create storage for cluster assignments
    if (m_assignments) {
        free(m_assignments);
        m_assignments = 0;
    }
    m_assignments = (int *)calloc(m_Ndata, sizeof(int));
    if (!m_assignments) {
        MYERROR("Could not allocate memory for assignments");
    }
}

/**
 * Internal method to compute mean vectors (centroids) for each cluster
 */
void KMeans::computeMeans() {

  // reset the old centroids
  resetMeans();
  
  if (m_debug) {
    cout << "Compute new mean vectors..." << endl;
  }
  // loop through pixels and accumulate centroids
  for (int y=0; y<m_height; y++) {
    int *assignment_offset_ptr = m_assignments + m_width*y;
    uchar *image_offset_ptr = (uchar*)(m_img->imageData + m_img->widthStep*y);
    for (int x=0; x<m_width; x++) {
        int clusterid = *(assignment_offset_ptr + x);

        if (clusterid < 1) {
          continue;
        }

        // get a pointer to pixel column of pixels
        unsigned index = 3*x;

        // get RGB values from the image
        unsigned char b = (image_offset_ptr)[index];
        unsigned char g = (image_offset_ptr)[index+1];
        unsigned char r = (image_offset_ptr)[index+2];

        // accumulate the vector components into the appropriate mean
        int cid = clusterid - 1;
        pixelVector pv = m_means[cid];

        // sum up individual components
        pv.x += x;
        pv.y += y;
        pv.b += b;
        pv.g += g;
        pv.r += r;
        pv.counter += 1;

        m_means[cid] = pv;
    }

  }

    // normalize the mean vectors
    for (int k=0; k<m_K; k++) {
        if (m_empty[k]) {    
            // once a mean becomes empty we skip it
            continue;
        }

        pixelVector pv = m_means[k];
        int n = pv.counter;

        if (n == 0) {
            m_empty[k] = 1;
            continue;
        }

        pv.x = pv.x / n;
        pv.y = pv.y / n;
        pv.b = pv.b / n;
        pv.g = pv.g / n;
        pv.r = pv.r / n;

        m_means[k] = pv;
        if (m_debug) {
            cout << "mean " << k+1 
                << " (x=" << pv.x 
                << ", y=" << pv.y 
                << ", r=" << (int)pv.r 
                << ", g=" << (int)pv.g 
                << ", b=" << (int)pv.b 
                << ", n=" << pv.counter << ")" << endl;
        }
    }
}

/**
 * Internal method to compute covariance and inverse covariance
 * for each cluster.
 * First when compute the covariance matrix, then we compute it's
 * determinant, then we normalize the covariance matrix to unit
 * determinant, then we invert the normalized covariance matrix.
 */
void KMeans::computeCovariances() {

    // reset the old (inverse) covariance matrix
    resetCovariances();
    
    if (m_debug) {
        cout << "Compute new covariance matrix..." << endl;
    }


    // loop through pixels and accumulate covariances
    for (int y=0; y<m_height; y++) {
        int *assignment_offset_ptr = m_assignments + m_width*y;
        uchar *image_offset_ptr = (uchar*)(m_img->imageData + m_img->widthStep*y);
        for (int x=0; x<m_width; x++) {

            int clusterid = *(assignment_offset_ptr + x);

            if (clusterid < 1) {
                continue;
            }
            // get a pointer to pixel column of pixels
            unsigned index = 3*x;

            // get RGB values from the image
            unsigned char b = (image_offset_ptr)[index];
            unsigned char g = (image_offset_ptr)[index+1];
            unsigned char r = (image_offset_ptr)[index+2];

            // get the mean vector associated with this pixel
            int cid = clusterid - 1;
            pixelVector pv = m_means[cid];

            // create 2 vectors from the pixel vector and the mean
            double m[NUM_DIMENSIONS];
            double p[NUM_DIMENSIONS];
            m[pixel_x] = pv.x;
            m[pixel_y] = pv.y;
            m[pixel_r] = pv.r;
            m[pixel_g] = pv.g;
            m[pixel_b] = pv.b;
            p[pixel_x] = x;
            p[pixel_y] = y;
            p[pixel_r] = r;
            p[pixel_g] = g;
            p[pixel_b] = b;
            
            for (int i=0; i<NUM_DIMENSIONS; i++) {
                for (int j=0; j<NUM_DIMENSIONS; j++) {
                    m_covariance[cid][i][j] += (p[i] - m[i])*(p[j] - m[j]);
                }
            }
        } // end for y
    } // end for x

    for (int k=0; k<m_K; k++) {
        int i;

        if (m_empty[k]) {    
            // once a mean becomes empty we skip it
            continue;
        }

        // normalize the covariances
        pixelVector pv = m_means[k];
        double Nminus1 = pv.counter - 1;
        for (i=0; i<NUM_DIMENSIONS; i++) {
            for (int j=0; j<NUM_DIMENSIONS; j++) {
                m_covariance[k][i][j] /= Nminus1;
            }
        }



        // perform LU decomp on the covariance matrix
        int indx[NUM_DIMENSIONS];
        double det;
        ludcmp(m_covariance[k], NUM_DIMENSIONS, indx, &det);
        // after this point m_covariance is altered into upper triangular

        // compute the determinant
        for (i=0; i<NUM_DIMENSIONS; i++) {
            det *= m_covariance[k][i][i];
        }

        // use this to normalize inverse covariance to it has det=1
        double norm = pow(det, 1/5);
        // compute the inverse covariance
        double col[NUM_DIMENSIONS];
        for (i=0; i<NUM_DIMENSIONS; i++) {
            int j;
            for (j=0; j<NUM_DIMENSIONS; j++) {
                col[j] = 0.0;
            }

            col[i] = 1.0;
            lubksb(m_covariance[k], NUM_DIMENSIONS, indx, col);
            for (j=0; j<NUM_DIMENSIONS; j++) {
                m_inverse_covariance[k][j][i] = col[j] * norm;
            }
        }

    }


}

/**
 * Internal method to free storage for means
 */
void KMeans::freeMeans() {
  if (m_empty) {
    free(m_empty);
    m_empty = 0;
  }
  if (m_means) {
    free(m_means);
    m_means = 0;
  }
}

/**
 * Internal method to free storage for covariance matrices
 */
void KMeans::freeCovariances() {
    for (int k=0; k<m_K; k++) {
        for (int i=0; i<NUM_DIMENSIONS; i++) {
            free(m_covariance[k][i]);
            free(m_inverse_covariance[k][i]);
        }
        free(m_covariance[k]);
        free(m_inverse_covariance[k]);
    }
    free(m_covariance);
    free(m_inverse_covariance);
}

/**
 * Internal method to initialize mean vectors to zero
 */
void KMeans::resetMeans() {
  if (m_debug) {
    cout << "Reset means to zero..." << endl;
  }

  memset(m_means, 0, m_K*sizeof(pixelVector));
}

/**
 * Internal method to set (inverse) covariance matrices to zero
 */
void KMeans::resetCovariances() {
    if (m_debug) {
        cout << "Reset covariance matrix to zero..." << endl;
    }

    for (int k=0; k<m_K; k++) {
        for (int i=0; i<NUM_DIMENSIONS; i++) {
            for (int j=0; j<NUM_DIMENSIONS; j++) {
                m_inverse_covariance[k][i][j] = 0.0;
                m_covariance[k][i][j] = 0.0;
            }
        }
    }
}


/**
 * Internal method to allocate mean vectors
 */
void KMeans::allocateMeans() {
  freeMeans();
  m_empty = (int *)calloc(m_K, sizeof(int));
  m_means = (pixelVector *)calloc(m_K, sizeof(pixelVector));
}

/**
 * Internal method to allocate storage for covariance matrices
 */
void KMeans::allocateCovariances() {
    m_covariance = (double ***)calloc(m_K, sizeof(double **));
    m_inverse_covariance = (double ***)calloc(m_K, sizeof(double **));
    for (int k=0; k<m_K; k++) {
        m_covariance[k] = (double **)calloc(NUM_DIMENSIONS, sizeof(double *));
        m_inverse_covariance[k] = (double **)calloc(NUM_DIMENSIONS, sizeof(double *));
        for (int i=0; i<NUM_DIMENSIONS; i++) {
            m_covariance[k][i] = (double *)calloc(NUM_DIMENSIONS, sizeof(double));
            m_inverse_covariance[k][i] = (double *)calloc(NUM_DIMENSIONS, sizeof(double));
        }
    }
}
/**
 * Internal method to find the nearest mean vector.  Computes the Mahanabolis distance
 * from the specified pixel to each of the mean vectors.
 *
 * @param  x  The x coordinate of the pixel
 * @param  y  The y coordinate of the pixel
 * @param  d  A return the distance to the nearest neighbor
 * @return  The index of the nearest mean vector
 */
int KMeans::findNearestMean(int x, int y, double *d) {
  int nearK = 0;
  double nearest = 0.0;
  
  // get a pointer to pixel column of pixels
  uchar *ptr = (uchar*)((m_img)->imageData + (m_img)->widthStep*y);
  unsigned index = 3*x;

  // get RGB values from the image
  unsigned char b = (ptr)[index];
  unsigned char g = (ptr)[index+1];
  unsigned char r = (ptr)[index+2];


  // loop over centroids
  for (int k=0; k<m_K; k++) {
    pixelVector pv = m_means[k];

    // compute a distance
    double distance = 0.0;
    double v1[NUM_DIMENSIONS];
    v1[pixel_x] = (x - pv.x);
    v1[pixel_y] = (y - pv.y);
    v1[pixel_r] = (r - pv.r);
    v1[pixel_g] = (g - pv.g);
    v1[pixel_b] = (b - pv.b);
    for (int i=0; i<NUM_DIMENSIONS; i++) {
        double temp = 0.0;
        for (int j=0; j<NUM_DIMENSIONS; j++) {
            temp += m_inverse_covariance[k][i][j]*v1[j];
        }
        distance += v1[i]*temp;
    }

    // keep track of nearest neighbor
    if (k == 0) {
      nearK = k;
      nearest = distance;
    }
    else if (distance < nearest) {
      nearK = k;
      nearest = distance;
    }
  }

    
  if (m_debug) {
    cout << "nearest: x=" << x << " y=" << y << " k=" << nearK
         << " dist=" << nearest << endl;
  }
  if (d) {
    *d = nearest;
  }
  return nearK;
}

/**
 * Internal method to give each document a random cluster assignment.
 * 
 */
void KMeans::randomInit() {
  if (m_debug) {
    cout << "Randomize initial clusters..." << endl;
  }
  if (m_Ndata < m_K) {
    MYERROR("Number of pixels is less than K");
  }

    // reset temporary storage for histogram of cluster assignments
    memset(m_distribution, 0, m_K*sizeof(int));

    // initialize list with cluster ids in sequence,
    // we will shuffle the list laster to achieve randomness
    for (int m=0; m<m_K; m++) {
        m_clusterlist[m] = m+1;
    }

    // randomly assign clusters (stratified).  we keep shuffling the list of clusters
    // like a deck of cards, draw a card to give the initial cluster assignment for a
    // given pixel, and then reshuffle the deck when we've drawn all the cards.  we
    // have one card per cluster
    int counter = m_K;
    for (int i=0; i<m_Ndata; i++) {
        if (counter >= m_K) {
            // time to randomly shuffle the cluster list
            shuffle(m_clusterlist, m_K);
            counter = 0;
        }

        // draw a cluster assignment
        int initcluster = m_clusterlist[counter++];

        if (initcluster < 1 || initcluster > m_K) {
            MYERROR("Bad initial cluster in KMeans::randomInit()");
        }

        // assign the cluster id to the pixel
        m_assignments[i] = initcluster;

        // tabulate the clusters that we've assigned (i.e. a histogram)
        m_distribution[initcluster - 1] += 1;
        if (m_debug) {
            cout << "initialize index=" << i
                << " cluster=" << m_assignments[i] << endl;
        }
    }
  
    // check for empty clusters
    int good = true;
    for (int k=0; k<m_K; k++) {
        if (m_distribution[k] < 1) {
            good = false;
            break;
        }
    }

    // try again if some clusters were empty
    if (!good) {
        MYERROR("Initial cluster is empty in KMeans::randomInit()");
    }
}

// return a random integer between 0 and l
int KMeans::random(int limit) {
  int irand = rand();
  if (m_debug) {
    //cout << "rand=" << irand << " RM=" << RAND_MAX << endl;
  }
  return (int)((double)m_K * irand / (double)(RAND_MAX + 1.0));
}

// randomly shuffle an array of integers
void KMeans::shuffle(int *array, int size) {
  for (int i=size-1; i>0; i--) {
    int j = random(i+1);
    if (i == j) {
      continue;
    }
    // swap elements
    int temp = array[i];
    array[i] = array[j];
    array[j] = temp;
  }
}

// from numerical recipes in C
// computes LU decomposition of a matrix
void KMeans::ludcmp(double **a, int n, int *indx, double *d) {
    int i, imax, j, k;
    double big, dum, sum, temp;
    
    *d = 1.0;
    for (i=0; i<n; i++) {
        big = 0.0;
        for (j=0; j<n; j++) {
            if ((temp = fabs(a[i][j])) > big) {
                big = temp;
            }
        }
        if (big == 0.0) {
            MYERROR("Singular matrix in KMeans::ludcmp");
        }
        vv[i] = 1.0/big;
    }
    
    for (j=0; j<n; j++) {
        for (i=0; i<j; i++) {
            sum = a[i][j];
            for (k=0; k<i; k++) {
                sum -= a[i][k]*a[k][j];
            }
            a[i][j] = sum;
        }
        big = 0.0;
        for (i=j; i<n; i++) {
            sum = a[i][j];
            for (k=0; k<j; k++) {
                sum -= a[i][k]*a[k][j];
            }
            a[i][j] = sum;
            if ((dum=vv[i]*fabs(sum)) >= big) {
                big = dum;
                imax = i;
            }
        }
        if (j != imax) {
            for (k=0; k<n; k++) {
                dum = a[imax][k];
                a[imax][k] = a[j][k];
                a[j][k] = dum;
            }
            *d = -(*d);
            vv[imax]=vv[j];
        }
        indx[j] = imax;
        if (a[j][j] == 0.0) {
            a[j][j] = 1.0e-20;
        }
        if (j != n) {
            dum = 1.0/(a[j][j]);
            for (i=j; i<n; i++) {
                a[i][j] *= dum;
            }
        }
    }
}

// from Numerical Recipes in C
void KMeans::lubksb(double **a, int n, int *indx, double *b) {
    int i, ii=0, ip, j;
    double sum;

    for (i=0; i<n; i++) {
        ip = indx[i];
        sum = b[ip];
        b[ip] = b[i];
        if (ii) {
            for (j=ii; j<i-1; j++) {
                sum -= a[i][j]*b[j];
            }
        }
        else if (sum) {
            ii = i;
        }
        b[i] = sum;
    }
    for (i=n-1; i>=0; i--) {
        sum = b[i];
        for (j=i+1; j<n; j++) {
            sum -= a[i][j]*b[i];
        }
        b[i] = sum/a[i][i];
    }
}
