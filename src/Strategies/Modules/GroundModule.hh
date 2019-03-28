#pragma once

class GroundModule : public Drawable {
public:
  Ground* ground;

  GroundModule(Ground* _ground) : ground(_ground) {}

  void draw() {
  }
};