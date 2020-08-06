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

    window.set_light(Light::StickToCamera);

    let mut ps = physics::Scene::new();

    let box_mesh = Rc::new(physics::unit_box_mesh());
    
    let scale = [1.0, 1.0, 1.0];
    let mut rb1 = physics::RigidBody::new_box(1.0, "red", scale[0], scale[1], scale[2], &box_mesh);
    let x: na::Vector3<f32> = Default::default();
    let q = na::UnitQuaternion::from_rotation_matrix(&na::Rotation3::from_euler_angles(0.0, na::RealField::frac_pi_4(), 0.0));
    rb1.set_pose(&x, &q);
    // rb1.l = na::Vector3::new(0.0, 0.0, 0.25);
    let rb1 = ps.add_rb(rb1);
    let mut c1      = window.add_cube(scale[0], scale[1], scale[2]);
    c1.data_mut().set_local_rotation(ps.get_rb(rb1).q());
    c1.data_mut().set_local_translation(na::Translation3::from(ps.get_rb(rb1).x()));
    c1.set_color(1.0, 0.0, 0.0);

    let mut rb2 = physics::RigidBody::new_box(1.0, "green", scale[0], scale[1], scale[2], &box_mesh);
    let x = na::Vector3::new(-2.5, 0.0, 0.0);
    let q = na::UnitQuaternion::from_rotation_matrix(&na::Rotation3::from_euler_angles(0.0, na::RealField::frac_pi_4(), na::RealField::frac_pi_4()));
    // let q = na::UnitQuaternion::from_euler_angles(0.7938401, -1.3559403, -2.985731);
    rb2.set_pose(&x, &q);
    rb2.p = na::Vector3::new(0.5, 0.0, 0.0);
    // rb2.l = na::Vector3::new(0.0, 0.5, 0.0);
    let rb2 = ps.add_rb(rb2);
    let mut c2      = window.add_cube(scale[0], scale[1], scale[2]);
    c2.data_mut().set_local_rotation(ps.get_rb(rb2).q());
    c2.data_mut().set_local_translation(na::Translation3::from(ps.get_rb(rb2).x()));
    c2.set_color(0.0, 1.0, 0.0);
    
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

        c1.data_mut().set_local_rotation(ps.get_rb(rb1).q());
        c1.data_mut().set_local_translation(na::Translation3::<f32>::from(ps.get_rb(rb1).x()));

        c2.data_mut().set_local_rotation(ps.get_rb(rb2).q());
        c2.data_mut().set_local_translation(na::Translation3::<f32>::from(ps.get_rb(rb2).x()));

        // let force = na::Vector3::new(0.05, 0.0, 0.0);
        let force = na::Vector3::new(0.0, 0.0, 0.0);
        // let torque = na::Vector3::new(0.0, 0.07, 0.0);
        let torque = na::Vector3::new(0.0, 0.0, 0.0);

        if !paused || do_step {            
            ps.update(dt);
            do_step = false;
            // println!("ITERATION {}", n);
            n = n + 1;
        }

        // draw_vertex_velocities(&rb1, &mut tmesh1, &mut window, &cyan);
        // draw_vertex_velocities(&rb2, &mut tmesh2, &mut window, &cyan);
    }
}