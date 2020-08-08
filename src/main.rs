use std::rc::Rc;
use rand::distributions::Uniform;
use std::iter::Iterator;
use std::fs::File;
use rand::SeedableRng;
use std::io::Write;

use nalgebra as na;

use kiss3d;
use kiss3d::window::Window;
use kiss3d::event::{Action, WindowEvent};
use kiss3d::light::Light;
use kiss3d::scene::SceneNode;
use kiss3d::camera::Camera;

use physics;

fn draw_face_normals(tmesh: &physics::TransformedMesh, window: &mut Window, vertex_color: &na::Point3<f32>) {
    for quad in tmesh.mesh.quads.iter() {
        let v0 = tmesh.get_vertex(quad[0]);
        let v1 = tmesh.get_vertex(quad[1]);
        let v2 = tmesh.get_vertex(quad[2]);
        let plane_n = (v1 - v0).cross(&(v2 - v0)).normalize();
        let plane_p = 0.25 * (v0 + v1 + v2 + tmesh.get_vertex(quad[3]));
        window.draw_line(&na::Point3::from(plane_p), &na::Point3::from(plane_p + plane_n), vertex_color);
    }
}

fn draw_edges(tmesh: &physics::TransformedMesh, window: &mut Window, vertex_color: &na::Point3<f32>) {
    for &(v1, v2) in tmesh.mesh.edges.iter() {
        window.draw_line(&na::Point3::from(tmesh.get_vertex(v1)), &na::Point3::from(tmesh.get_vertex(v2)), vertex_color);
        window.draw_line(&na::Point3::from(tmesh.get_vertex(v1)), &na::Point3::from(tmesh.get_vertex(v2)), vertex_color);
    }
}

fn draw_vertices(tmesh: &physics::TransformedMesh, window: &mut Window, vertex_color: &na::Point3<f32>) {
    for v in 0..tmesh.mesh.vertices.len() {
        window.draw_point(&na::Point3::from(tmesh.get_vertex(v)), vertex_color);
    }
}

fn draw_separating_plane_info(sep_plane: physics::SeparatingPlane, tmesh1: &physics::TransformedMesh, tmesh2: &physics::TransformedMesh,
                              window: &mut Window, vertex_color: &na::Point3<f32>) {
    match sep_plane {
        physics::SeparatingPlane::Mesh1Face(f, v_idx) => {
            let fv0 = tmesh1.get_vertex(f[0]);
            let fv1 = tmesh1.get_vertex(f[1]);
            let fv2 = tmesh1.get_vertex(f[2]);
            let fv3 = tmesh1.get_vertex(f[3]);
            let plane_n = (fv1 - fv0).cross(&(fv2 - fv0)).normalize();
            let plane_p = fv0;
            let v = tmesh2.get_vertex(v_idx);
            let p = physics::closest_point_on_plane_to_point(&v, &plane_p, &plane_n);
            // window.draw_line(&na::Point3::from(p), &na::Point3::from(v), &vertex_color);
            window.draw_line(&na::Point3::from(p), &na::Point3::from(p + plane_n), &vertex_color);
            
            window.draw_point(&na::Point3::from(fv0), &vertex_color);
            window.draw_point(&na::Point3::from(fv1), &vertex_color);
            window.draw_point(&na::Point3::from(fv2), &vertex_color);
            window.draw_point(&na::Point3::from(fv3), &vertex_color);
        },
        physics::SeparatingPlane::Mesh2Face(f, v_idx) => {
            let fv0 = tmesh2.get_vertex(f[0]);
            let fv1 = tmesh2.get_vertex(f[1]);
            let fv2 = tmesh2.get_vertex(f[2]);
            let fv3 = tmesh2.get_vertex(f[3]);
            let plane_n = (fv1 - fv0).cross(&(fv2 - fv0)).normalize();
            let plane_p = fv0;
            let v = tmesh1.get_vertex(v_idx);
            let p = physics::closest_point_on_plane_to_point(&v, &plane_p, &plane_n);
            //window.draw_line(&na::Point3::from(p), &na::Point3::from(v), &vertex_color);
            window.draw_line(&na::Point3::from(p), &na::Point3::from(p + plane_n), &vertex_color);

            window.draw_point(&na::Point3::from(fv0), &vertex_color);
            window.draw_point(&na::Point3::from(fv1), &vertex_color);
            window.draw_point(&na::Point3::from(fv2), &vertex_color);
            window.draw_point(&na::Point3::from(fv3), &vertex_color);
        },
        physics::SeparatingPlane::Edges((m1v1, m1v2), (m2v1, m2v2)) => {
            let (v1, v2) = physics::closest_points_on_edges(
                &tmesh1.get_vertex(m1v1),& tmesh1.get_vertex(m1v2), &tmesh2.get_vertex(m2v1), &tmesh2.get_vertex(m2v2));
            window.draw_line(&na::Point3::from(v1), &na::Point3::from(v2), &vertex_color);
            // window.draw_line(&na::Point3::from(v1), &na::Point3::from((v2 - v1).normalize()), &vertex_color);
            // let e1 = tmesh1.get_vertex(m1v2) - tmesh1.get_vertex(m1v1);
            // let e2 = tmesh2.get_vertex(m2v2) - tmesh2.get_vertex(m2v1);
            // let n = e1.cross(&e2);
            // window.draw_line(&na::Point3::from(v1), &na::Point3::from(v1 + 0.5*n), &vertex_color);
            window.draw_line(&na::Point3::from(tmesh1.get_vertex(m1v1)), &na::Point3::from(tmesh1.get_vertex(m1v2)), &vertex_color);
            window.draw_line(&na::Point3::from(tmesh2.get_vertex(m2v1)), &na::Point3::from(tmesh2.get_vertex(m2v2)), &vertex_color);
        }
    };
}

fn draw_vertex_velocities(rb: &physics::RigidBody, tmesh: &physics::TransformedMesh, window: &mut Window, color: &na::Point3<f32>) {
    for i in 0..tmesh.mesh.vertices.len() {
        let p = tmesh.get_vertex(i);
        let v_dir = rb.get_point_velocity(&tmesh.get_vertex(i)).normalize();
        window.draw_line(&na::Point3::from(p), &na::Point3::from(p + v_dir*0.25), color);
    }
}

fn make_room(dim: f32, mesh: &Rc<physics::Mesh>, physics_scene: &mut physics::Scene, window: &mut Window, only_render_floor: bool) {
    for &offset in &[-0.5*dim - 0.5, 0.5*dim + 0.5] {
        for coord in 0..3 {
            let mut scale = na::Vector3::new(dim, dim, dim);
            scale[coord] = 1.0;
            let mut rb = physics::RigidBody::new_static_box("wall", scale[0], scale[1], scale[2], mesh);
            let mut p = na::Vector3::<f32>::zeros();
            p[coord] = offset;
            rb.set_pose(&p, &Default::default());
            if !only_render_floor || (coord == 1 && offset < 0.0) {
                let mut c = window.add_cube(scale[0], scale[1], scale[2]);
                c.data_mut().set_local_rotation(rb.q());
                c.data_mut().set_local_translation(na::Translation3::from(rb.x()));
                c.set_color(0.8, 0.8, 0.8);
            }
            physics_scene.add_rb(rb);
        }
    }
}

fn aabb_overlap(p1: &na::Vector3<f32>, scale1: &na::Vector3<f32>, p2: &na::Vector3<f32>, scale2: &na::Vector3<f32>) -> bool {
    for coord in 0..3 {
        if p1[coord] - 0.5*scale1[coord] > p2[coord] + 0.5*scale2[coord] ||
           p1[coord] + 0.5*scale1[coord] < p2[coord] - 0.5*scale2[coord] {
            return false;
        }
    }
    return true;
}

fn make_random_boxes(n: usize, room_dim: f32, box_dim: f32, mesh: &Rc<physics::Mesh>, physics_scene: &mut physics::Scene, window: &mut Window) -> Vec<(usize, SceneNode)> {
    let mut rng = rand::rngs::StdRng::seed_from_u64(2);
    let bounding_dim = box_dim * 2.0f32.sqrt() + 0.1;
    let rng_dist = Uniform::new(-0.5*room_dim + 0.5*bounding_dim, 0.5*room_dim - 0.5*bounding_dim);
    let mut bounding_boxes = vec![];
    let scale = na::Vector3::<f32>::new(bounding_dim, bounding_dim, bounding_dim);
    for _ in 0..n {
        let mut p;
        loop {
            p = na::Vector3::from_distribution(&rng_dist, &mut rng);
            if bounding_boxes.iter().all(|rhs| !aabb_overlap(&p, &scale, rhs, &scale)) {
                break;
            }
        }
        bounding_boxes.push(p);
    }
    let mut boxes = vec![];
    let color_dist = Uniform::new(0.0, 1.0);
    let angle_dist = Uniform::new(0.0, 2.0*std::f32::consts::PI);
    for (i, b) in bounding_boxes.iter().enumerate() {
        let mut rb = physics::RigidBody::new_box(1.0, &format!("box{}", i), box_dim, box_dim, box_dim, mesh);
        let rand_angles = na::Vector3::from_distribution(&angle_dist, &mut rng);
        let q = na::UnitQuaternion::<f32>::from_euler_angles(rand_angles[0], rand_angles[1], rand_angles[2]);
        rb.set_pose(&b, &q);
        let mut c = window.add_cube(box_dim, box_dim, box_dim);
        c.data_mut().set_local_rotation(rb.q());
        c.data_mut().set_local_translation(na::Translation3::from(rb.x()));
        let rand_color = na::Vector3::from_distribution(&color_dist, &mut rng);
        c.set_color(rand_color[0], rand_color[1], rand_color[2]);
        boxes.push((physics_scene.add_rb(rb), c));
    }
    return boxes;
}

fn make_boxes(mesh: &Rc<physics::Mesh>, physics_scene: &mut physics::Scene, window: &mut Window) -> Vec<(usize, SceneNode)> {
    let scale = [1.0, 1.0, 1.0];
    let mut rb1 = physics::RigidBody::new_box(1.0, "red", scale[0], scale[1], scale[2], &mesh);
    let x: na::Vector3<f32> = Default::default();
    let q = na::UnitQuaternion::from_rotation_matrix(&na::Rotation3::from_euler_angles(0.0, na::RealField::frac_pi_4(), 0.0));
    rb1.set_pose(&x, &q);
    // rb1.l = na::Vector3::new(0.0, 0.0, 0.25);
    let rb1 = physics_scene.add_rb(rb1);
    let mut c1      = window.add_cube(scale[0], scale[1], scale[2]);
    c1.data_mut().set_local_rotation(physics_scene.get_rb(rb1).q());
    c1.data_mut().set_local_translation(na::Translation3::from(physics_scene.get_rb(rb1).x()));
    c1.set_color(1.0, 0.0, 0.0);

    let mut rb2 = physics::RigidBody::new_box(1.0, "green", scale[0], scale[1], scale[2], &mesh);
    let x = na::Vector3::new(-2.5, 0.0, 0.0);
    let q = na::UnitQuaternion::from_rotation_matrix(&na::Rotation3::from_euler_angles(0.0, na::RealField::frac_pi_4(), na::RealField::frac_pi_4()));
    // let q = na::UnitQuaternion::from_euler_angles(0.7938401, -1.3559403, -2.985731);
    rb2.set_pose(&x, &q);
    rb2.p = na::Vector3::new(5.0, 0.0, 0.0);
    // rb2.l = na::Vector3::new(0.0, 0.5, 0.0);
    let rb2 = physics_scene.add_rb(rb2);
    let mut c2      = window.add_cube(scale[0], scale[1], scale[2]);
    c2.data_mut().set_local_rotation(physics_scene.get_rb(rb2).q());
    c2.data_mut().set_local_translation(na::Translation3::from(physics_scene.get_rb(rb2).x()));
    c2.set_color(0.0, 1.0, 0.0);

    let mut rb3 = physics::RigidBody::new_box(1.0, "blue", scale[0], scale[1], scale[2], &mesh);
    let x = na::Vector3::new(2.0, 0.0, 0.0);
    let q = na::UnitQuaternion::from_rotation_matrix(&na::Rotation3::from_euler_angles(0.0, na::RealField::frac_pi_4(), na::RealField::frac_pi_4()));
    // let q = na::UnitQuaternion::from_euler_angles(0.7938401, -1.3559403, -2.985731);
    rb3.set_pose(&x, &q);
    // rb3.p = na::Vector3::new(0.5, 0.0, 0.0);
    // rb2.l = na::Vector3::new(0.0, 0.5, 0.0);
    let rb3 = physics_scene.add_rb(rb3);
    let mut c3      = window.add_cube(scale[0], scale[1], scale[2]);
    c3.data_mut().set_local_rotation(physics_scene.get_rb(rb3).q());
    c3.data_mut().set_local_translation(na::Translation3::from(physics_scene.get_rb(rb3).x()));
    c3.set_color(0.0, 0.0, 1.0);

    return vec![(rb1, c1), (rb2, c2), (rb3, c3)];
}

fn dump_state(physics_scene: &physics::Scene) {
    let mut f = File::create("dump.txt").unwrap();
    for i in 0..physics_scene.get_num_rbs() {
        let rb = physics_scene.get_rb(i);
        f.write_all(format!("============ {} ===========\n", rb.name).as_bytes()).unwrap();
        f.write_all(format!("x: {}", rb.x()).as_bytes()).unwrap();
        let eulers = rb.q().euler_angles();
        let eulers = na::Vector3::<f32>::new(eulers.0, eulers.1, eulers.2);
        f.write_all(format!("q: {}", eulers).as_bytes()).unwrap();
        f.write_all(format!("p: {}", rb.p).as_bytes()).unwrap();
        f.write_all(format!("l: {}", rb.l).as_bytes()).unwrap();
        f.write_all(format!("active: {}\n", rb.active).as_bytes()).unwrap();
    }
}

fn main() {
    let mut window = Window::new("Kiss3d: cube");
    println!("window size: {} {}", window.width(), window.height());
    let eye = na::Point3::new(10.0f32, 10.0, 10.0);
    let at = na::Point3::origin();
    let mut camera = kiss3d::camera::ArcBall::new(eye, at);

    window.set_light(Light::StickToCamera);

    let mut ps = physics::Scene::new();

    let box_mesh = Rc::new(physics::unit_box_mesh());

    const ROOM_SIZE: f32 = 10.0;

    make_room(ROOM_SIZE, &box_mesh, &mut ps, &mut window, true);

    let mut boxes = make_random_boxes(7, ROOM_SIZE, 1.0, &box_mesh, &mut ps, &mut window);
    
    let fps: u64 = 60; 
    let dt: physics::Float = 1.0 / (fps as physics::Float);
    println!("dt: {}", dt);
    window.set_framerate_limit(Some(fps));
    
    let cyan = na::Point3::new(0.0, 1.0, 1.0);
    let blue = na::Point3::new(0.0, 0.0, 1.0);

    window.set_point_size(2.0);
    
    let mut paused = true;
    let mut do_step = false;
    let mut n = 1;
    let mut show_names = false;
    while window.render_with_camera(&mut camera) {
        for mut event in window.events().iter() {
            match event.value {
                WindowEvent::Key(kiss3d::event::Key::Space, Action::Press, _) => {
                    paused = !paused;
                },
                WindowEvent::Key(kiss3d::event::Key::Right, Action::Press, _) => {
                    if paused {
                        do_step = true;
                    }
                },
                WindowEvent::Key(kiss3d::event::Key::D, Action::Press, _) => {
                    dump_state(&ps);
                },
                WindowEvent::Key(kiss3d::event::Key::N, Action::Press, _) => {
                    show_names = !show_names;
                }
                WindowEvent::Scroll(xshift, yshift, _) => {
                    event.inhibited = true;
                    camera.set_dist(camera.dist() + 0.1 * (yshift as f32));
                }
                _ => {}
            }
        }

        if show_names {
            for i in 0..ps.get_num_rbs() {
                let x = ps.get_rb(i).x();
                // project gives values from (0,0) to (size.0, size.1), but with bottom-left corner being 0,0. However,
                // draw_text for some reason says the window has extends of 2*size and 0,0 is in the top-left corner.
                let mut x2d = camera.project(&na::Point3::from(x), &na::Vector2::new(2.0 * window.width() as f32, 2.0 * window.height() as f32));
                x2d.y = 2.0 * (window.height() as f32) - x2d.y;
                window.draw_text(&ps.get_rb(i).name, &na::Point2::from(x2d), 50.0, &kiss3d::text::Font::default(), &na::Point3::new(1.0, 1.0, 1.0));
            }
        }

        if !paused || do_step {            
            ps.update(dt);
            do_step = false;
            // println!("ITERATION {}", n);
            n = n + 1;
        }

        for b in boxes.iter_mut() {
            b.1.data_mut().set_local_rotation(ps.get_rb(b.0).q());
            b.1.data_mut().set_local_translation(na::Translation3::<f32>::from(ps.get_rb(b.0).x()));
        }

        // draw_vertex_velocities(&rb1, &mut tmesh1, &mut window, &cyan);
        // draw_vertex_velocities(&rb2, &mut tmesh2, &mut window, &cyan);
    }
}