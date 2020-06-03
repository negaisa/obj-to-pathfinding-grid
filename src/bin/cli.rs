use nalgebra::Vector3;
use obj::Obj;
use obj_to_pathfinding_grid;
use obj_to_pathfinding_grid::{NoOpPreprocessor, Progress};
use std::path::PathBuf;
use structopt::StructOpt;

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

    let progress = StdOutProgress::new();

    println!("Starting to convert obj file");

    let obj = Obj::load(input).expect("Failed to load input file");

    obj_to_pathfinding_grid::convert(
        &obj,
        center,
        scale,
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
