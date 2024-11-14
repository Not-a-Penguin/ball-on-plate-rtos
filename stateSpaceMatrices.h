
static const int systemOrder = 2;

struct ss{
  Matrix<systemOrder,systemOrder> A;
  Matrix<systemOrder,1> B;
  Matrix<1,systemOrder> C;
  Matrix<1,1> D;
};

//Ts = 35ms
ss sys = {
  {
    1, 0.0349,
    0,  1
  },
  {
    -0.0019,
    -0.1078
  },
  {1, 0},
  {0}
};
