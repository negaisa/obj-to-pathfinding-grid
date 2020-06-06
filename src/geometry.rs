use nalgebra::Vector3;

/// Local vector represents grid coordinates from 0 to width/height.
#[derive(Debug, Eq, PartialEq)]
pub struct LocalVector {
    pub x: u32,
    pub y: u32,
    pub z: u32,
}

impl LocalVector {
    pub fn new(x: u32, y: u32, z: u32) -> Self {
        LocalVector { x, y, z }
    }

    /// Converts world vector to grid local vector.
    /// If world vector out of grid bounds will be set border coordinates.
    pub fn from_world_vector(
        vector: &Vector3<f32>,
        center: &Vector3<f32>,
        width: u32,
        height: u32,
    ) -> Self {
        let diff = center - vector;

        let half_width = width / 2;
        let half_height = height / 2;

        let mx = (half_width as i32 - diff.x.round() as i32).max(0) as u32;
        let my = (half_width as i32 - diff.y.round() as i32).max(0) as u32;
        let mz = (half_height as i32 - diff.z.round() as i32).max(0) as u32;

        let x = mx.min(width) as u32;
        let y = my.min(width) as u32;
        let z = mz.min(height) as u32;

        LocalVector { x, y, z }
    }

    pub fn to_world_vector(&self, center: &Vector3<f32>, width: u32, height: u32) -> Vector3<i32> {
        let half_width = width / 2;
        let half_height = height / 2;

        let min_x = (center.x.round() as i32) - half_width as i32;
        let min_y = (center.y.round() as i32) - half_width as i32;
        let min_z = (center.z.round() as i32) - half_height as i32;

        let x = min_x + self.x as i32;
        let y = min_y + self.y as i32;
        let z = min_z + self.z as i32;

        Vector3::new(x, y, z)
    }
}

#[derive(Debug)]
pub struct BoundingBox {
    pub min: Vector3<f32>,
    pub max: Vector3<f32>,
}

impl BoundingBox {
    pub fn new(min: Vector3<f32>, max: Vector3<f32>) -> Self {
        BoundingBox { min, max }
    }

    pub fn center(&self) -> Vector3<f32> {
        (self.min + self.max) / 2.0
    }

    pub fn width(&self) -> f32 {
        let width_x = self.max.x - self.min.x;
        let width_y = self.max.y - self.min.y;

        width_x.max(width_y)
    }

    pub fn height(&self) -> f32 {
        self.max.z - self.min.z
    }
}

#[derive(Debug)]
pub struct Triangle {
    pub a: Vector3<f32>,
    pub b: Vector3<f32>,
    pub c: Vector3<f32>,
}

impl Triangle {
    pub fn new(a: Vector3<f32>, b: Vector3<f32>, c: Vector3<f32>) -> Self {
        Triangle { a, b, c }
    }
}

impl Triangle {
    pub fn is_inside(&self, vector: &Vector3<i32>) -> bool {
        let box_center = Vector3::new(vector.x as f32, vector.y as f32, vector.z as f32);
        let box_half_size = Vector3::new(0.5, 0.5, 0.5);

        // Move the triangle so that the box is centered around the origin.
        let v0 = self.a - &box_center;
        let v1 = self.b - &box_center;
        let v2 = self.c - &box_center;

        // The edges of the triangle.
        let e0 = v1 - &v0;
        let e1 = v2 - &v1;
        let e2 = v0 - &v2;

        // 1. Test the AABB against the minimal AABB around the triangle.
        if min_max_overlaps(box_half_size.x, v0.x, v1.x, v2.x) {
            return false;
        }

        if min_max_overlaps(box_half_size.y, v0.y, v1.y, v2.y) {
            return false;
        }

        if min_max_overlaps(box_half_size.z, v0.z, v1.z, v2.z) {
            return false;
        }

        // 2. Test if the box intersects the plane of the triangle.
        let normal = e0.cross(&e1);
        let d = -normal.dot(&v0);

        let mut v_min = Vector3::new(0.0, 0.0, 0.0);
        let mut v_max = Vector3::new(0.0, 0.0, 0.0);

        if normal.x > 0.0 {
            v_min.x -= box_half_size.x;
            v_max.x += box_half_size.x;
        } else {
            v_min.x += box_half_size.x;
            v_max.x -= box_half_size.x;
        }

        if normal.y > 0.0 {
            v_min.y -= box_half_size.y;
            v_max.y += box_half_size.y;
        } else {
            v_min.y += box_half_size.y;
            v_max.y -= box_half_size.y;
        }

        if normal.z > 0.0 {
            v_min.z -= box_half_size.z;
            v_max.z += box_half_size.z;
        } else {
            v_min.z += box_half_size.z;
            v_max.z -= box_half_size.z;
        }

        if normal.dot(&v_min) + d > 0.0 {
            return false;
        }

        if normal.dot(&v_max) + d < 0.0 {
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

    pub fn scale(self, scale: f32) -> Triangle {
        let a = self.a.scale(scale);
        let b = self.b.scale(scale);
        let c = self.c.scale(scale);

        Triangle::new(a, b, c)
    }

    pub fn bounding_box(&self) -> BoundingBox {
        let min_x = self.a.x.min(self.b.x).min(self.c.x) - 1.0;
        let min_y = self.a.y.min(self.b.y).min(self.c.y) - 1.0;
        let min_z = self.a.z.min(self.b.z).min(self.c.z) - 1.0;

        let max_x = self.a.x.max(self.b.x).max(self.c.x) + 1.0;
        let max_y = self.a.y.max(self.b.y).max(self.c.y) + 1.0;
        let max_z = self.a.z.max(self.b.z).max(self.c.z) + 1.0;

        let min = Vector3::new(min_x, min_y, min_z);
        let max = Vector3::new(max_x, max_y, max_z);

        BoundingBox::new(min, max)
    }
}

fn min_max_overlaps(box_half_size: f32, v0: f32, v1: f32, v2: f32) -> bool {
    let min = v0.min(v1).min(v2);
    let max = v0.max(v1).max(v2);

    min > box_half_size || max < -box_half_size
}

fn axis_test_zy(
    point1: &Vector3<f32>,
    point2: &Vector3<f32>,
    box_half_size: &Vector3<f32>,
    edge: &Vector3<f32>,
) -> bool {
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

fn axis_test_mzx(
    point1: &Vector3<f32>,
    point2: &Vector3<f32>,
    box_half_size: &Vector3<f32>,
    edge: &Vector3<f32>,
) -> bool {
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

fn axis_test_yx(
    point1: &Vector3<f32>,
    point2: &Vector3<f32>,
    box_half_size: &Vector3<f32>,
    edge: &Vector3<f32>,
) -> bool {
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
    use crate::geometry::{BoundingBox, LocalVector, Triangle};
    use nalgebra::Vector3;

    #[test]
    fn test_is_inside() {
        let a = Vector3::new(0.0, 0.0, 0.0);
        let b = Vector3::new(5.0, 5.0, 5.0);
        let c = Vector3::new(-5.0, 5.0, -5.0);

        let triangle = Triangle::new(a, b, c);

        assert!(triangle.is_inside(&Vector3::new(0, 0, 0)));
        assert!(triangle.is_inside(&Vector3::new(5, 5, 5)));
        assert!(!triangle.is_inside(&Vector3::new(-1, -1, -1)));
        assert!(!triangle.is_inside(&Vector3::new(6, 6, 6)));
        assert!(!triangle.is_inside(&Vector3::new(10, 5, 0)));
        assert!(!triangle.is_inside(&Vector3::new(-3, 6, -2)));
    }

    #[test]
    fn test_scale() {
        let a = Vector3::new(0.0, 0.0, 0.0);
        let b = Vector3::new(5.0, 5.0, 5.0);
        let c = Vector3::new(-5.0, 5.0, -5.0);

        let triangle = Triangle::new(a, b, c);
        let scaled_triangle = triangle.scale(2.0);

        let scaled_a = Vector3::new(0.0, 0.0, 0.0);
        let scaled_b = Vector3::new(10.0, 10.0, 10.0);
        let scaled_c = Vector3::new(-10.0, 10.0, -10.0);

        assert_eq!(scaled_triangle.a, scaled_a);
        assert_eq!(scaled_triangle.b, scaled_b);
        assert_eq!(scaled_triangle.c, scaled_c);
    }

    #[test]
    fn test_bounding_box() {
        let a = Vector3::new(0.0, 0.0, 0.0);
        let b = Vector3::new(5.0, 5.0, 5.0);
        let c = Vector3::new(-5.0, 5.0, -5.0);

        let triangle = Triangle::new(a, b, c);
        let bounding_box = triangle.bounding_box();

        let min = Vector3::new(-6.0, -1.0, -6.0);
        let max = Vector3::new(6.0, 6.0, 6.0);

        assert_eq!(bounding_box.min, min);
        assert_eq!(bounding_box.max, max);
    }

    #[test]
    fn test_bounding_box2() {
        let a = Vector3::new(0.0, 0.0, 0.0);
        let b = Vector3::new(5.0, 5.0, 0.0);
        let c = Vector3::new(-5.0, -5.0, 0.0);

        let triangle = Triangle::new(a, b, c);
        let bounding_box = triangle.bounding_box();

        let min = Vector3::new(-6.0, -6.0, -1.0);
        let max = Vector3::new(6.0, 6.0, 1.0);

        assert_eq!(bounding_box.min, min);
        assert_eq!(bounding_box.max, max);
    }

    #[test]
    fn test_from_world_vector() {
        fn to_local_vector(vector: &Vector3<f32>) -> LocalVector {
            LocalVector::from_world_vector(vector, &Vector3::new(0.0, 0.0, 0.0), 500, 500)
        }

        let local1 = to_local_vector(&Vector3::new(0.0, 0.0, 0.0));
        let local2 = to_local_vector(&Vector3::new(250.0, 250.0, 250.0));
        let local3 = to_local_vector(&Vector3::new(-250.0, -250.0, -250.0));
        let local4 = to_local_vector(&Vector3::new(150.0, 150.0, 150.0));
        let local5 = to_local_vector(&Vector3::new(-150.0, -250.0, -150.0));
        let local6 = to_local_vector(&Vector3::new(-251.0, -251.0, -251.0));
        let local7 = to_local_vector(&Vector3::new(251.0, 251.0, 251.0));

        assert_eq!(local1, LocalVector::new(250, 250, 250));
        assert_eq!(local2, LocalVector::new(500, 500, 500));
        assert_eq!(local3, LocalVector::new(0, 0, 0));
        assert_eq!(local4, LocalVector::new(400, 400, 400));
        assert_eq!(local5, LocalVector::new(100, 0, 100));
        assert_eq!(local6, LocalVector::new(0, 0, 0));
        assert_eq!(local7, LocalVector::new(500, 500, 500));
    }

    #[test]
    fn test_from_world_vector2() {
        fn to_local_vector(vector: &Vector3<f32>) -> LocalVector {
            LocalVector::from_world_vector(
                vector,
                &Vector3::new(366.666656, -13866.666016, 84.290909),
                500,
                500,
            )
        }

        let local1 = to_local_vector(&Vector3::new(366.666656, -13866.666016, 84.290909));
        let local2 = to_local_vector(&Vector3::new(404.166656, -14024.999023, 22.343037));

        assert_eq!(local1, LocalVector::new(250, 250, 250));
        assert_eq!(local2, LocalVector::new(288, 92, 188));
    }

    #[test]
    fn test_to_world_vector() {
        fn from_local_vector(vector: &LocalVector) -> Vector3<i32> {
            vector.to_world_vector(&Vector3::new(0.0, 0.0, 0.0), 500, 500)
        }

        let local1 = from_local_vector(&LocalVector::new(250, 250, 250));
        let local2 = from_local_vector(&LocalVector::new(500, 500, 500));
        let local3 = from_local_vector(&LocalVector::new(0, 0, 0));
        let local4 = from_local_vector(&LocalVector::new(400, 400, 400));
        let local5 = from_local_vector(&LocalVector::new(100, 0, 100));

        assert_eq!(local1, Vector3::new(0, 0, 0));
        assert_eq!(local2, Vector3::new(250, 250, 250));
        assert_eq!(local3, Vector3::new(-250, -250, -250));
        assert_eq!(local4, Vector3::new(150, 150, 150));
        assert_eq!(local5, Vector3::new(-150, -250, -150));
    }

    #[test]
    fn test_to_world_vector2() {
        fn from_local_vector(vector: &LocalVector) -> Vector3<i32> {
            vector.to_world_vector(
                &Vector3::new(366.666656, -13866.666016, 84.290909),
                500,
                500,
            )
        }

        let local1 = from_local_vector(&LocalVector::new(250, 250, 250));
        let local2 = from_local_vector(&LocalVector::new(288, 92, 188));

        assert_eq!(local1, Vector3::new(367, -13867, 84));
        assert_eq!(local2, Vector3::new(405, -14025, 22));
    }

    #[test]
    fn test_to_world_vector3() {
        fn from_local_vector(vector: &LocalVector) -> Vector3<i32> {
            vector.to_world_vector(&Vector3::new(200.0, 200.0, 75.0), 400, 150)
        }

        let local1 = from_local_vector(&LocalVector::new(0, 0, 0));
        let local2 = from_local_vector(&LocalVector::new(400, 400, 150));

        assert_eq!(local1, Vector3::new(0, 0, 0));
        assert_eq!(local2, Vector3::new(400, 400, 150));
    }

    #[test]
    fn test_bounding_box_center() {
        let min = Vector3::new(-200.0, -200.0, -200.0);
        let max = Vector3::new(200.0, 200.0, 200.0);

        let bounding_box = BoundingBox::new(min, max);

        assert_eq!(bounding_box.center(), Vector3::new(0.0, 0.0, 0.0));
    }

    #[test]
    fn test_bounding_box_width() {
        let min = Vector3::new(-100.0, -200.0, -200.0);
        let max = Vector3::new(100.0, 200.0, 200.0);

        let bounding_box = BoundingBox::new(min, max);

        assert_eq!(bounding_box.width(), 400.0);
    }

    #[test]
    fn test_bounding_box_height() {
        let min = Vector3::new(-100.0, -200.0, -200.0);
        let max = Vector3::new(100.0, 200.0, 200.0);

        let bounding_box = BoundingBox::new(min, max);

        assert_eq!(bounding_box.height(), 400.0);
    }
}
