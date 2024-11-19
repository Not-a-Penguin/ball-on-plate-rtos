
static const int systemOrder = 2;

struct ss{
  BLA::Matrix<systemOrder,systemOrder> A;
  BLA::Matrix<systemOrder,1> B;
  BLA::Matrix<1,systemOrder> C;
  BLA::Matrix<1,1> D;
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
