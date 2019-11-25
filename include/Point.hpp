#ifndef INCLUDE_POINT_HPP_
#define INCLUDE_POINT_HPP_

class Point {
	private:
		double x, y;

	public:
		explicit Point(double startX, double startY);

		double getX();

		double getY();
}

#endif //  INCLUDE_POINT_HPP
