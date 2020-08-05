use std::rc::Rc;

use kiss3d;
use nalgebra as na;

use kiss3d::window::Window;
use kiss3d::event::{Action, WindowEvent};
use kiss3d::light::Light;

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

fn main() {
    let mut window = Window::new("Kiss3d: cube");
    let eye = na::Point3::new(10.0f32, 10.0, 10.0);
    let at = na::Point3::origin();
    let mut camera = kiss3d::camera::ArcBall::new(eye, at);
    // let mut camera = kiss3d::camera::FirstPerson::new(eye, at);
    // camera.set_move_step(0.1);

    window.set_light(Light::StickToCamera);

    let box_mesh = Rc::new(physics::unit_box_mesh());
    
    let mut rb1 = physics::RigidBody::new_box(1.0, "red", 1.0, 1.0, 1.0, &box_mesh);
    let x: na::Vector3<f32> = Default::default();
    let q = na::UnitQuaternion::from_rotation_matrix(&na::Rotation3::from_euler_angles(0.0, na::RealField::frac_pi_4(), 0.0));
    rb1.set_pose(&x, &q);
    // rb1.l = na::Vector3::new(0.0, 0.0, 0.25);
    let mut c1      = window.add_cube(rb1.scale[0], rb1.scale[1], rb1.scale[2]);
    c1.data_mut().set_local_rotation(rb1.q());
    c1.data_mut().set_local_translation(na::Translation3::from(rb1.x()));
    c1.set_color(1.0, 0.0, 0.0);

    let mut rb2 = physics::RigidBody::new_box(1.0, "green", 1.0, 1.0, 1.0, &box_mesh);
    let x = na::Vector3::new(-2.5, 0.0, 0.0);
    let q = na::UnitQuaternion::from_rotation_matrix(&na::Rotation3::from_euler_angles(0.0, na::RealField::frac_pi_4(), na::RealField::frac_pi_4()));
    // let q = na::UnitQuaternion::from_euler_angles(0.7938401, -1.3559403, -2.985731);
    rb2.set_pose(&x, &q);
    rb2.p = na::Vector3::new(0.5, 0.0, 0.0);
    // rb2.l = na::Vector3::new(0.0, 0.5, 0.0);
    let mut c2      = window.add_cube(rb2.scale[0], rb2.scale[1], rb2.scale[2]);
    c2.data_mut().set_local_rotation(rb2.q());
    c2.data_mut().set_local_translation(na::Translation3::from(rb2.x()));
    c2.set_color(0.0, 1.0, 0.0);
    
    let fps: u64 = 60; 
    let dt: physics::Float = 1.0 / (fps as physics::Float);
    println!("dt: {}", dt);
    window.set_framerate_limit(Some(fps));
    
    let cyan = na::Point3::new(0.0, 1.0, 1.0);
    let blue = na::Point3::new(0.0, 0.0, 1.0);

    window.set_point_size(2.0);
    
    let mut initial_guess = None;
    let mut contact_info = None;
    let mut paused = true;
    let mut do_step = false;
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
                WindowEvent::Scroll(xshift, yshift, _) => {
                    event.inhibited = true;
                    camera.set_dist(camera.dist() + 0.1 * (yshift as f32));
                }
                _ => {}
            }
        }

        c1.data_mut().set_local_rotation(rb1.q());
        c1.data_mut().set_local_translation(na::Translation3::<f32>::from(rb1.x()));

        c2.data_mut().set_local_rotation(rb2.q());
        c2.data_mut().set_local_translation(na::Translation3::<f32>::from(rb2.x()));

        // let force = na::Vector3::new(0.05, 0.0, 0.0);
        let force = na::Vector3::new(0.0, 0.0, 0.0);
        // let torque = na::Vector3::new(0.0, 0.07, 0.0);
        let torque = na::Vector3::new(0.0, 0.0, 0.0);

        if !paused || do_step {
            // initial_guess = physics::update(&mut rb1, &mut tmesh1, &mut rb2, &mut tmesh2, initial_guess, dt);
            let update_info = physics::update(&mut rb1, &mut rb2, initial_guess, dt);
            initial_guess = Some(update_info.sep_plane);
            contact_info = update_info.contact_info;
            do_step = false;
        }

        // draw_vertex_velocities(&rb1, &mut tmesh1, &mut window, &cyan);
        // draw_vertex_velocities(&rb2, &mut tmesh2, &mut window, &cyan);

        // if initial_guess.is_some() {
        //     draw_separating_plane_info(initial_guess.unwrap(), &mut tmesh1, &mut tmesh2, &mut window, &cyan);
        // }

        if contact_info.is_some() {
            let contact_info = contact_info.unwrap();
            window.draw_line(&na::Point3::from(contact_info.contact_p), &na::Point3::from(contact_info.contact_p + contact_info.impulse.normalize()), &cyan);
        }

        // rb1.update(&na::Vector3::new(0.0, 0.0, 0.0), &na::Vector3::new(0.0, 0.0, 0.0), dt);
        // tmesh1.set_transform(rb1.get_transform());

        // rb2.update(&force, &torque, dt);
        // tmesh2.set_transform(rb2.get_transform());

        // let sep_plane = physics::find_separating_plane(&mut tmesh1, &mut tmesh2, initial_guess);
        // if sep_plane.is_none() {
        //     println!("{}", rb2.x);
        //     let e = rb2.q.euler_angles();
        //     println!("COLLISION: {} {} {}", e.0, e.1, e.2);
        // } else {
        //     let sep_plane = sep_plane.unwrap().0;
        //     initial_guess = Some(sep_plane);
        //     draw_separating_plane_info(sep_plane, &mut tmesh1, &mut tmesh2, &mut window, &cyan);
        // }
    }
}