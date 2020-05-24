use crate::geometry::Triangle;
use flying_pathfinding::Grid;
use nalgebra::Vector3;
use obj::Obj;
use std::path::PathBuf;
use structopt::StructOpt;

mod geometry;

#[derive(StructOpt, Debug)]
#[structopt(name = "obj-to-pathfinding-grid")]
struct Opt {
    #[structopt(short, long, parse(from_os_str))]
    input: PathBuf,
    #[structopt(short, long, parse(from_os_str))]
    output: Option<PathBuf>,
    #[structopt(short, long)]
    width: u32,
    #[structopt(short, long)]
    height: u32,
    #[structopt(short, long, default_value = "1.0")]
    scale: f32,
    #[structopt(short = "x", long, default_value = "0.0")]
    center_x: f32,
    #[structopt(short = "y", long, default_value = "0.0")]
    center_y: f32,
    #[structopt(short = "z", long, default_value = "0.0")]
    center_z: f32,
}

fn main() {
    let opt: Opt = Opt::from_args();

    let input = &opt.input;
    let center = Vector3::new(opt.center_x, opt.center_y, opt.center_z);
    let scale = opt.scale;
    let width = opt.width;
    let height = opt.height;

    let output = match &opt.output {
        Some(v) => v.clone(),
        None => {
            let input_name_without_extension = input.file_stem().unwrap().to_str().unwrap();
            let output_name = format!("{}.{}", input_name_without_extension, "dat");

            PathBuf::from(output_name)
        }
    };

    let std_out_progress = StdOutProgress {};

    convert(
        input,
        &output,
        center,
        scale,
        width,
        height,
        std_out_progress,
    );
}

trait Progress {
    fn starting(&self);

    fn update_progress(&self, percent: f32);

    fn finished(&self);
}

struct StdOutProgress {}

impl Progress for StdOutProgress {
    fn starting(&self) {
        println!("Starting!")
    }

    fn update_progress(&self, percent: f32) {
        print!("Current progress: {:.2}%\r", percent)
    }

    fn finished(&self) {
        println!("\nFinished!")
    }
}

fn convert<P: Progress>(
    input: &PathBuf,
    output: &PathBuf,
    center: Vector3<f32>,
    scale: f32,
    width: u32,
    height: u32,
    progress: P,
) {
    progress.starting();

    let obj = Obj::load(input).expect("Failed to load input file");

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

    grid.export(output).expect("Failed to save output file");
    progress.finished();
}

fn parse_triangles(obj: Obj) -> Vec<Triangle> {
    let data = obj.data;
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

    let min_x = min.x.floor() as u32;
    let max_x = max.x.ceil() as u32;

    let min_y = min.y.floor() as u32;
    let max_y = max.y.ceil() as u32;

    let min_z = min.z.floor() as u32;
    let max_z = max.z.ceil() as u32;

    let mut obstacles = Vec::new();

    for x in min_x..max_x.min(width) {
        for y in min_y..max_y.min(width) {
            for z in min_z..max_z.min(height) {
                let point = Vector3::new(x, y, z);

                if triangle.is_inside(&point) {
                    obstacles.push(point);
                }
            }
        }
    }

    obstacles
}
