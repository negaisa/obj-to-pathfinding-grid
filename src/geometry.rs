#[derive(Debug)]
pub struct Point {
    pub x: f32,
    pub y: f32,
    pub z: f32
}

impl Point {
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Point {
            x,
            y,
            z
        }
    }

    pub fn subtract(&self, other: &Point) -> Point {
        let x = self.x - other.x;
        let y = self.y - other.y;
        let z = self.z - other.z;

        Point { x, y, z }
    }

    pub fn scalar(&self, other: &Point) -> f32 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }
}

pub trait Geometry {
    fn is_inside(&self, point: &Point) -> bool;
}

#[derive(Debug)]
pub struct Triangle {
    pub a: Point,
    pub b: Point,
    pub c: Point,
}

impl Triangle {
    pub fn new(a: Point, b: Point, c: Point) -> Self {
        Triangle {
            a,
            b,
            c
        }
    }
}

impl Geometry for Triangle {
    fn is_inside(&self, point: &Point) -> bool {
        let s1 = self.c.subtract(&self.a);
        let s2 = self.b.subtract(&self.a);
        let s3 = point.subtract(&self.a);

        let d11 = s1.scalar(&s1);
        let d12 = s1.scalar(&s2);
        let d13 = s1.scalar(&s3);
        let d22 = s2.scalar(&s2);
        let d23 = s2.scalar(&s3);

        let d = 1.0 / (d11 * d22 - d12 * d12);
        let w1 = (d22 * d13 - d12 * d23) * d;
        let w2 = (d11 * d23 - d12 * d13) * d;

        w1 >= 0.0 && w2 >= 0.0 && (w1 + w2) < 1.0
    }
}

#[cfg(test)]
mod tests {
    use crate::geometry::{Point, Triangle, Geometry};

    #[test]
    fn test_triangle_is_inside() {
        let a = Point::new(0.0, 0.0, 0.0);
        let b = Point::new(5.0, 5.0, 5.0);
        let c = Point::new(-5.0, 5.0, -5.0);

        let triangle = Triangle::new(a, b, c);

        assert!(triangle.is_inside(&Point::new(0.0, 0.0, 0.0)));
        assert!(triangle.is_inside(&Point::new(4.99, 4.99, 4.99)));
        assert!(triangle.is_inside(&Point::new(-4.99, 4.99, -4.99)));

        assert!(!triangle.is_inside(&Point::new(-3.0, 6.0, -2.0)));
        assert!(!triangle.is_inside(&Point::new(5.0, 5.0, 5.0)));
        assert!(!triangle.is_inside(&Point::new(-5.0, 5.0, -5.0)));
    }

}