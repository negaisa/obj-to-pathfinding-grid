use nalgebra::Vector3;
use obj::Obj;
use obj_to_pathfinding_grid;
use obj_to_pathfinding_grid::geometry::Triangle;
use obj_to_pathfinding_grid::parse_triangles;
use obj_to_pathfinding_grid::{bounding_box, NoOpPreprocessor, Progress};
use std::fs;
use std::path::{Path, PathBuf};
use structopt::StructOpt;

const DEFAULT_OUTPUT_FOLDER: &str = "grid";

#[derive(StructOpt, Debug)]
#[structopt(name = "obj-to-pathfinding-grid")]
struct Opt {
    #[structopt(short, long, parse(from_os_str))]
    input: PathBuf,
    #[structopt(short, long, parse(from_os_str))]
    output: Option<PathBuf>,
    #[structopt(short, long)]
    width: Option<u32>,
    #[structopt(short, long)]
    height: Option<u32>,
    #[structopt(short, long)]
    scale: Option<f32>,
    #[structopt(short = "x", long)]
    center_x: Option<f32>,
    #[structopt(short = "y", long)]
    center_y: Option<f32>,
    #[structopt(short = "z", long)]
    center_z: Option<f32>,
}

fn main() {
    let opt: Opt = Opt::from_args();

    let input = &opt.input;

    let obj = Obj::load(input).expect("Failed to load input file");
    let scale = opt.scale.unwrap_or(1.0);

    let triangles: Vec<Triangle> = parse_triangles(&obj)
        .into_iter()
        .map(|t| t.scale(scale))
        .collect();

    let bounding_box = bounding_box(&triangles);
    let bounding_box_center = bounding_box.center();

    let center_x = opt.center_x.unwrap_or(bounding_box_center.x);
    let center_y = opt.center_y.unwrap_or(bounding_box_center.y);
    let center_z = opt.center_z.unwrap_or(bounding_box_center.z);

    let center = Vector3::new(center_x, center_y, center_z);

    let width = opt.width.unwrap_or(bounding_box.width() as u32);
    let height = opt.height.unwrap_or(bounding_box.height() as u32);

    let output = match &opt.output {
        Some(v) => v.clone(),
        None => {
            let input_name_without_extension = input.file_stem().unwrap().to_str().unwrap();

            let output_folder = Path::new(DEFAULT_OUTPUT_FOLDER);

            if !output_folder.exists() {
                fs::create_dir(output_folder).expect("Failed to create output folder");
            }

            let output_name = format!("{}.{}", input_name_without_extension, "dat");
            output_folder.join(PathBuf::from(output_name))
        }
    };

    let progress = StdOutProgress::new();

    println!("Starting to convert obj file");

    obj_to_pathfinding_grid::convert(
        triangles,
        center,
        width,
        height,
        progress,
        NoOpPreprocessor {},
    )
    .export(output)
    .expect("Failed to save output file");

    print!("\nFinished converting obj to grid");
}

struct StdOutProgress {}

impl StdOutProgress {
    pub fn new() -> Self {
        StdOutProgress {}
    }
}

impl Progress for StdOutProgress {
    fn update_progress(&self, percent: f32) {
        print!("Current progress: {:.2}%\r", percent)
    }
}
