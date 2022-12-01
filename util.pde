// Normalize an angle to be between 0 and TAU (= 2 PI)
float normalize_angle(float angle)
{
    while (angle < 0) angle += TAU;
    while (angle > TAU) angle -= TAU;
    return angle;
}

// Normalize an angle to be between -PI and PI
float normalize_angle_left_right(float angle)
{
    while (angle < -PI) angle += TAU;
    while (angle > PI) angle -= TAU;
    return angle;
}

PVector averagePVectors(List<PVector> list) {
  PVector result = new PVector();
  for(PVector v : list) {
      result = PVector.add(result, v);
  }
  result = PVector.div(result, list.size());
  return result;
}

PVector headingToGlobal(float heading, PVector basis) {
   return PVector.add(basis, PVector.fromAngle(heading)); 
}

PVector headingToGlobalMag(float heading, PVector basis, float mag) {
   return PVector.add(basis, PVector.fromAngle(heading).setMag(mag)); 
}

void drawPVectorLine(PVector start, PVector end) {
   line(start.x, start.y, end.x, end.y); 
}
