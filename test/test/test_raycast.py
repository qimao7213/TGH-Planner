import math

# 辅助函数

def signum(x):
    return 0 if x == 0 else -1 if x < 0 else 1

def mod(value, modulus):
    return (value % modulus + modulus) % modulus

def intbound(s, ds):
    # Find the smallest positive t such that s + t * ds is an integer.
    if ds == 0:
        return float('inf')  # 如果没有移动，则返回无穷大
    if ds < 0:
        return intbound(-s, -ds)
    else:
        s = mod(s, 1)
        return (1 - s) / ds

# RayCaster 类

class RayCaster:
    def __init__(self):
        self.start_ = None
        self.end_ = None
        self.x_ = self.y_ = self.z_ = 0
        self.endX_ = self.endY_ = self.endZ_ = 0
        self.dx_ = self.dy_ = self.dz_ = 0
        self.stepX_ = self.stepY_ = self.stepZ_ = 0
        self.tMaxX_ = self.tMaxY_ = self.tMaxZ_ = 0
        self.tDeltaX_ = self.tDeltaY_ = self.tDeltaZ_ = 0
        self.dist_ = 0
        self.step_num_ = 0
        self.maxDist_ = 0

    def setInput(self, start, end):
        self.start_ = start
        self.end_ = end
        self.x_ = int(math.floor(start[0]))
        self.y_ = int(math.floor(start[1]))
        self.z_ = int(math.floor(start[2]))
        self.endX_ = int(math.floor(end[0]))
        self.endY_ = int(math.floor(end[1]))
        self.endZ_ = int(math.floor(end[2]))
        direction = [end[i] - start[i] for i in range(3)]
        self.maxDist_ = sum(d**2 for d in direction)

        self.dx_ = self.endX_ - self.x_
        self.dy_ = self.endY_ - self.y_
        self.dz_ = self.endZ_ - self.z_

        self.stepX_ = signum(self.dx_)
        self.stepY_ = signum(self.dy_)
        self.stepZ_ = signum(self.dz_)

        self.tMaxX_ = intbound(start[0], self.dx_)
        self.tMaxY_ = intbound(start[1], self.dy_)
        self.tMaxZ_ = intbound(start[2], self.dz_)

        self.tDeltaX_ = abs(1 / self.dx_) if self.dx_ != 0 else float('inf')
        self.tDeltaY_ = abs(1 / self.dy_) if self.dy_ != 0 else float('inf')
        self.tDeltaZ_ = abs(1 / self.dz_) if self.dz_ != 0 else float('inf')

        if self.stepX_ == 0 and self.stepY_ == 0 and self.stepZ_ == 0:
            return False
        else:
            return True

    def step(self):
        print(f"Step {self.step_num_}: Position = ({self.x_}, {self.y_}, {self.z_}), process = ({self.tMaxX_}, {self.tMaxY_}, {self.tMaxY_})")
        
        if (self.x_, self.y_, self.z_) == (self.endX_, self.endY_, self.endZ_):
            return False

        if self.tMaxX_ < self.tMaxY_:
            if self.tMaxX_ < self.tMaxZ_:
                self.x_ += self.stepX_
                self.tMaxX_ += self.tDeltaX_
            else:
                self.z_ += self.stepZ_
                self.tMaxZ_ += self.tDeltaZ_
        else:
            if self.tMaxY_ < self.tMaxZ_:
                self.y_ += self.stepY_
                self.tMaxY_ += self.tDeltaY_
            else:
                self.z_ += self.stepZ_
                self.tMaxZ_ += self.tDeltaZ_

        self.step_num_ += 1
        return True

# 命令行测试

def main():
    start = tuple(map(float, input("Enter start coordinates (x, y, z): ").split()))
    end = tuple(map(float, input("Enter end coordinates (x, y, z): ").split()))

    ray_caster = RayCaster()
    if not ray_caster.setInput(start, end):
        print("Ray initialization failed; start and end points may be the same.")
        return

    print("Starting ray traversal...")
    while ray_caster.step():
        pass
    print("Ray traversal complete.")

if __name__ == "__main__":
    main()


