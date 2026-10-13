#ifndef SAME_SIDE_TRIPLET_PREPARATION_HPP
#define SAME_SIDE_TRIPLET_PREPARATION_HPP

namespace RS_marching::geometry {

struct SameSideTripletPreparation {
  bool parent_contact = false;
  double parent_bearing = 0.0;
};

static_assert(sizeof(SameSideTripletPreparation) == 16);

} // namespace RS_marching::geometry

#endif
