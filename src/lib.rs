use crate::geometry::{LocalVector, Triangle};
use flying_pathfinding::Grid;
use nalgebra::Vector3;
use obj::Obj;

pub mod geometry;

pub trait Progress {
    fn update_progress(&self, percent: f32);
}

pub trait Preprocessor {
    fn pre_process(
        &self,
        triangle: Triangle,
        width: u32,
        height: u32,
        center: Vector3<f32>,
    ) -> Option<Triangle>;
}

pub struct NoOpPreprocessor {}

impl Preprocessor for NoOpPreprocessor {
    fn pre_process(
        &self,
        triangle: Triangle,
        _width: u32,
        _height: u32,
        _center: Vector3<f32>,
    ) -> Option<Triangle> {
        Some(triangle)
    }
}

pub fn convert<Prg: Progress, Pre: Preprocessor>(
    obj: &Obj,
    center: Vector3<f32>,
    scale: f32,
    width: u32,
    height: u32,
    progress: Prg,
    preprocessor: Pre,
) -> Grid {
    let triangles: Vec<Triangle> = parse_triangles(&obj)
        .into_iter()
        .map(|t| t.scale(scale))
        .collect();

    let mut obstacles = Vec::new();
    let length = triangles.len();
    let mut current = 0;

    for triangle in triangles {
        let processed_triangle_opt = preprocessor.pre_process(triangle, width, height, center);

        if let Some(processed_triangle) = processed_triangle_opt {
            obstacles.extend(find_obstacles(&processed_triangle, &center, width, height));
        }

        current += 1;

        let percent = current as f32 * 100.0 / length as f32;
        progress.update_progress(percent);
    }

    let mut grid = Grid::new(width, height);

    for obstacle in obstacles {
        grid.set_obstacle(obstacle.x, obstacle.y, obstacle.z);
    }

    grid
}

fn parse_triangles(obj: &Obj) -> Vec<Triangle> {
    let data = &obj.data;
    let positions = &data.position;

    data.objects
        .iter()
        .flat_map(|obj| &obj.groups)
        .flat_map(|group| &group.polys)
        .map(|poly| &poly.0)
        .filter(|indices| indices.len() == 3)
        .map(|indices| {
            let position1 = positions[indices[0].0];
            let position2 = positions[indices[1].0];
            let position3 = positions[indices[2].0];

            let a = Vector3::new(position1[0], position1[1], position1[2]);
            let b = Vector3::new(position2[0], position2[1], position2[2]);
            let c = Vector3::new(position3[0], position3[1], position3[2]);

            Triangle::new(a, b, c)
        })
        .collect()
}

/// To find obstacles we check every point in triangle bounding box.
fn find_obstacles(
    triangle: &Triangle,
    center: &Vector3<f32>,
    width: u32,
    height: u32,
) -> Vec<LocalVector> {
    let bounding_box = triangle.bounding_box();

    // Convert bounding box to local coordinates.
    let min = LocalVector::from_world_vector(&bounding_box.min, &center, width, height);
    let max = LocalVector::from_world_vector(&bounding_box.max, &center, width, height);

    let mut obstacles = Vec::new();

    for x in min.x..max.x {
        for y in min.y..max.y {
            for z in min.z..max.z {
                let local_vector = LocalVector::new(x, y, z);

                // Triangle coordinates are global.
                let global_vector = local_vector.to_world_vector(&center, width, height);

                if triangle.is_inside(&global_vector) {
                    obstacles.push(local_vector);
                }
            }
        }
    }

    obstacles
}
