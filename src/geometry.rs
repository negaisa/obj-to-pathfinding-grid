use std::panic;

#[derive(Debug)]
pub struct Point {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Point {
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Point { x, y, z }
    }

    pub fn splat(value: f32) -> Self {
        Point::new(value, value, value)
    }

    pub fn multiply(&self, other: &Point) -> Point {
        let x = self.x * other.x;
        let y = self.y * other.y;
        let z = self.z * other.z;

        Point::new(x, y, z)
    }

    pub fn cross(&self, other: &Point) -> Point {
        let x = self.y * other.z - self.z * other.y;
        let y = self.z * other.x - self.x * other.z;
        let z = self.x * other.y - self.y * other.x;

        Point::new(x, y, z)
    }

    pub fn add(&self, other: &Point) -> Point {
        let x = self.x + other.x;
        let y = self.y + other.y;
        let z = self.z + other.z;

        Point::new(x, y, z)
    }

    pub fn subtract(&self, other: &Point) -> Point {
        let x = self.x - other.x;
        let y = self.y - other.y;
        let z = self.z - other.z;

        Point::new(x, y, z)
    }

    pub fn scalar(&self, other: &Point) -> f32 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    pub fn get_point(&self, index: usize) -> f32 {
        match index {
            0 => self.x,
            1 => self.y,
            2 => self.z,
            _ => panic!("Index of bounds"),
        }
    }

    pub fn set_point(&self, index: usize, value: f32) -> Point {
        match index {
            0 => Point::new(self.x + value, self.y, self.z),
            1 => Point::new(self.x, self.y + value, self.z),
            2 => Point::new(self.x, self.y, self.z + value),
            _ => panic!("Index of bounds"),
        }
    }
}

pub trait Geometry {
    fn is_inside(&self, point: &Point) -> bool;

    fn recalculate(&self, center: &Point, scale: f32) -> Self;
}

#[derive(Debug)]
pub struct Triangle {
    pub a: Point,
    pub b: Point,
    pub c: Point,
}

impl Triangle {
    pub fn new(a: Point, b: Point, c: Point) -> Self {
        Triangle { a, b, c }
    }
}

impl Geometry for Triangle {
    fn is_inside(&self, point: &Point) -> bool {
        let box_min = Point::new(point.x.floor(), point.y.floor(), point.z.floor());
        let box_half_size = Point::splat(0.5);
        let box_center = box_min.add(&box_half_size);

        // Move the triangle so that the box is centered around the origin.
        let v0 = self.a.subtract(&box_center);
        let v1 = self.b.subtract(&box_center);
        let v2 = self.c.subtract(&box_center);

        // The edges of the triangle.
        let e0 = v1.subtract(&v0);
        let e1 = v2.subtract(&v1);
        let e2 = v0.subtract(&v2);

        // 1. Test the AABB against the minimal AABB around the triangle.
        for i in 0..3 {
            if min_max_overlaps(
                box_half_size.get_point(i),
                v0.get_point(i),
                v1.get_point(i),
                v2.get_point(i),
            ) {
                return false;
            }
        }

        // 2. Test if the box intersects the plane of the triangle.
        let normal = e0.cross(&e1);
        let d = -normal.scalar(&v0);

        let mut v_min = Point::splat(0.0);
        let mut v_max = Point::splat(0.0);

        for i in 0..3 {
            if normal.get_point(i) > 0.0 {
                v_min = v_min.set_point(i, -box_half_size.get_point(i));
                v_max = v_max.set_point(i, box_half_size.get_point(i));
            } else {
                v_min = v_min.set_point(i, box_half_size.get_point(i));
                v_max = v_max.set_point(i, -box_half_size.get_point(i));
            }
        }

        if normal.scalar(&v_min) + d > 0.0 {
            return false;
        }

        if normal.scalar(&v_max) < 0.0 {
            return false;
        }

        // 3. Axis test

        // Edge #1
        if !axis_test_zy(&v0, &v2, &box_half_size, &e0) {
            return false;
        }

        if !axis_test_mzx(&v0, &v2, &box_half_size, &e0) {
            return false;
        }

        if !axis_test_yx(&v1, &v2, &box_half_size, &e0) {
            return false;
        }

        // Edge #2
        if !axis_test_zy(&v0, &v2, &box_half_size, &e1) {
            return false;
        }

        if !axis_test_mzx(&v0, &v2, &box_half_size, &e1) {
            return false;
        }

        if !axis_test_yx(&v0, &v1, &box_half_size, &e1) {
            return false;
        }

        // Edge #3
        if !axis_test_zy(&v0, &v1, &box_half_size, &e2) {
            return false;
        }

        if !axis_test_mzx(&v0, &v1, &box_half_size, &e2) {
            return false;
        }

        if !axis_test_yx(&v1, &v2, &box_half_size, &e2) {
            return false;
        }

        true
    }

    fn recalculate(&self, center: &Point, scale: f32) -> Self {
        let scale_point = Point::new(scale, scale, scale);

        let a = self.a.multiply(&scale_point).add(&center);
        let b = self.b.multiply(&scale_point).add(&center);
        let c = self.c.multiply(&scale_point).add(&center);

        Triangle::new(a, b, c)
    }
}

fn min_max_overlaps(box_half_size: f32, v0: f32, v1: f32, v2: f32) -> bool {
    let min = v0.min(v1).min(v2);
    let max = v0.max(v1).max(v2);

    min > box_half_size || max < -box_half_size
}

fn axis_test_zy(point1: &Point, point2: &Point, box_half_size: &Point, edge: &Point) -> bool {
    axis_test(
        edge.z,
        edge.y,
        point1.y,
        point1.z,
        point2.y,
        point2.z,
        box_half_size.y,
        box_half_size.z,
        false,
    )
}

fn axis_test_mzx(point1: &Point, point2: &Point, box_half_size: &Point, edge: &Point) -> bool {
    axis_test(
        -edge.z,
        edge.x,
        point1.x,
        point1.z,
        point2.x,
        point2.z,
        box_half_size.x,
        box_half_size.z,
        true,
    )
}

fn axis_test_yx(point1: &Point, point2: &Point, box_half_size: &Point, edge: &Point) -> bool {
    axis_test(
        edge.y,
        edge.x,
        point1.x,
        point1.y,
        point2.x,
        point2.y,
        box_half_size.x,
        box_half_size.y,
        false,
    )
}

fn axis_test(
    edge_axis1: f32,
    edge_axis2: f32,
    point1_axis1: f32,
    point1_axis2: f32,
    point2_axis1: f32,
    point2_axis2: f32,
    box_half_size_axis1: f32,
    box_half_size_axis2: f32,
    sign: bool,
) -> bool {
    let p1 = if sign {
        edge_axis1 * point1_axis1 + edge_axis2 * point1_axis2
    } else {
        edge_axis1 * point1_axis1 - edge_axis2 * point1_axis2
    };

    let p2 = if sign {
        edge_axis1 * point2_axis1 + edge_axis2 * point2_axis2
    } else {
        edge_axis1 * point2_axis1 - edge_axis2 * point2_axis2
    };

    let min = p1.min(p2);
    let max = p1.max(p2);

    let radius = edge_axis1.abs() * box_half_size_axis1 + edge_axis2.abs() * box_half_size_axis2;

    if min > radius || max < -radius {
        return false;
    }

    true
}

#[cfg(test)]
mod tests {
    use crate::geometry::{Geometry, Point, Triangle};

    #[test]
    fn test_is_inside() {
        let a = Point::new(0.0, 0.0, 0.0);
        let b = Point::new(5.0, 5.0, 5.0);
        let c = Point::new(-5.0, 5.0, -5.0);

        let triangle = Triangle::new(a, b, c);

        assert!(triangle.is_inside(&Point::new(0.0, 0.0, 0.0)));
        assert!(triangle.is_inside(&Point::new(5.0, 5.0, 5.0)));
        assert!(triangle.is_inside(&Point::new(-5.0, 5.0, -5.0)));

        assert!(!triangle.is_inside(&Point::new(-3.0, 6.0, -2.0)));
        assert!(!triangle.is_inside(&Point::new(10.0, 5.0, 0.0)));
    }
}
