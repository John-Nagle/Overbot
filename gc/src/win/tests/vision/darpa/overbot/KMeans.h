#ifndef _KMEANS
#define _KMEANS

#include <cv.h>

#define NUM_DIMENSIONS 5
#define CONVERGENCE_FACTOR 0.001
#define MAX_ITERATIONS 5

// use to map matrix indexes to x,y,r,g,b
enum pixelIndex {
    pixel_x,
    pixel_y,
    pixel_r,
    pixel_g,
    pixel_b
};
// use to represent mean vectors
struct pixelVector {
    int x;              // the x coordinate
    int y;              // the y coordinate
    unsigned r;    // the Red value
    unsigned g;    // the Green value
    unsigned b;    // the Blue value 
    int counter;        // the number of pixels in the mean
};


class KMeans
{
 private:
  // private members 
  int m_Ndim;
  int m_Ndata;
  int m_debug;
  int m_verbose;
  int m_K;
  int m_maxiterations;
  int *m_empty;
  int m_height;
  int m_width;
  IplImage *m_img;
  pixelVector *m_means;
  int *m_assignments;
  int *m_distribution;
  int *m_clusterlist;
  double ***m_inverse_covariance;
  double ***m_covariance;
  double *vv;

  // private methods
  void initialize();
  double assign();
  void freeMeans();
  void freeCovariances();
  void computeMeans();
  void computeCovariances();
  void resetMeans();
  void resetCovariances();
  void allocateMeans();
  void allocateCovariances();
  void randomInit();
  int random(int limit);
  void shuffle(int *list, int size);
  int findNearestMean(int x, int y, double *s=0);
  void resize(int w, int h);
    void ludcmp(double **a, int n, int *indx, double *d);
    void lubksb(double **a, int n, int *indx, double *b);

public:
  KMeans(int K);
  ~KMeans();

  void cluster(IplImage *img);
  int getAssignment(int x, int y);
  IplImage *getColorAssignments();

  void setDebug(int d);
  void setVerbose(int v);
  void setMaxIterations(int i);
};
#endif // _KMEANS

