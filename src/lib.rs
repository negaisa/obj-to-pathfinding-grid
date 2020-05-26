use crate::geometry::Triangle;
use flying_pathfinding::Grid;
use nalgebra::Vector3;
use obj::Obj;

pub mod geometry;

pub trait Progress {
    fn update_progress(&self, percent: f32);
}

pub fn convert<P: Progress>(
    obj: &Obj,
    center: Vector3<f32>,
    scale: f32,
    width: u32,
    height: u32,
    progress: P,
) -> Grid {
    let triangles: Vec<Triangle> = parse_triangles(obj)
        .into_iter()
        .map(|t| t.scale(scale))
        .map(|t| t.move_to(&center))
        .collect();

    let mut obstacles = Vec::new();
    let length = triangles.len();
    let mut current = 0;

    for triangle in triangles {
        obstacles.extend(find_obstacles(&triangle, width, height));
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

fn find_obstacles(triangle: &Triangle, width: u32, height: u32) -> Vec<Vector3<u32>> {
    let bounding_box = triangle.bounding_box();

    let min = bounding_box.min;
    let max = bounding_box.max;

    let min_x = (min.x.floor() as u32).min(width);
    let max_x = (max.x.ceil() as u32).max(width);

    let min_y = (min.y.floor() as u32).min(width);
    let max_y = (max.y.ceil() as u32).max(width);

    let min_z = (min.z.floor() as u32).min(height);
    let max_z = (max.z.ceil() as u32).max(height);

    let mut obstacles = Vec::new();

    for x in min_x..max_x {
        for y in min_y..max_y {
            for z in min_z..max_z {
                let point = Vector3::new(x, y, z);

                if triangle.is_inside(&point) {
                    obstacles.push(point);
                }
            }
        }
    }

    obstacles
}
