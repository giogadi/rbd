use std::rc::Rc;
use std::cell::RefCell;
use std::collections::HashMap;
use std::fmt;

use nalgebra as na;

pub type Float = f32;

pub enum IntegrationScheme {
    ExplicitEuler, SemiImplicitEuler
}

pub struct Scene {
    rbs: Vec<RigidBody>,
    prev_sep_planes: HashMap<(usize, usize), SeparatingPlane>,
    pub enable_gravity: bool,
    pub integration_scheme: IntegrationScheme
}

impl Scene {
    pub fn new() -> Scene {
        Scene {
            rbs: vec![],
            prev_sep_planes: HashMap::new(),
            enable_gravity: true,
            integration_scheme: IntegrationScheme::SemiImplicitEuler
        }
    }

    pub fn add_rb(&mut self, rb: RigidBody) -> usize {
        self.rbs.push(rb);
        return self.rbs.len() - 1;
    }

    // TODO: Can we do this without cloning?
    pub fn update(&mut self, dt_in: Float) -> (Float, Vec<ContactInfo>) {
        let mut latest_no_collision_t = 0.0;
        let mut earliest_collision_t = dt_in;
        let mut dt = dt_in;
        // TODO: do we need to copy the separating planes???
        let mut new_rbs = self.rbs.clone();
        let mut prev_collisions: Vec<(usize, usize)> = vec![];
        // TODO this gravity is FAKE! It assumes the RB has a mass of 1.0.
        let gravity;
        if self.enable_gravity {
            gravity = na::Vector3::new(0.0, -9.81, 0.0);
        } else {
            gravity = na::Vector3::new(0.0, 0.0, 0.0);
        }
        let mut contact_infos = vec![];
        loop {
            // DEBUG DO NOT SUBMIT.
            self.prev_sep_planes.clear();
            for rb1_idx in 0..new_rbs.len() {
                for rb2_idx in (rb1_idx + 1)..new_rbs.len() {
                    let new_sep_plane = find_separating_plane(new_rbs[rb1_idx].tmesh(), new_rbs[rb2_idx].tmesh(), None);
                    self.prev_sep_planes.insert((rb1_idx, rb2_idx), new_sep_plane.unwrap().0);
                }
            }

            for rb in new_rbs.iter_mut() {
                if rb.active {
                    let gravity = if rb.m_inv > 0.0 { gravity / rb.m_inv } else { na::Vector3::zeros() };
                    match self.integration_scheme {
                        IntegrationScheme::ExplicitEuler => rb.explicit_euler_update(&gravity, &Default::default(), dt),
                        IntegrationScheme::SemiImplicitEuler => rb.semi_implicit_euler_update(&gravity, &Default::default(), dt)
                    }
                }
            }
            let mut collisions = vec![];
            for rb1_idx in 0..new_rbs.len() {
                for rb2_idx in (rb1_idx + 1)..new_rbs.len() {
                    // TODO: This AABB pre-check causes us to not find sep planes when we need them in the hashmap
                    //
                    // TODO: need to use the max of the rb scale components because these scales are in body frame!!!
                    // let scale1 = new_rbs[rb1_idx].scale * ((2.0 as Float).sqrt() + 0.01);
                    // let scale2 = new_rbs[rb2_idx].scale * ((2.0 as Float).sqrt() + 0.01);
                    // if !aabb_overlap(&new_rbs[rb1_idx].x(), &scale1, &new_rbs[rb2_idx].x(), &scale2) {
                    //     continue;
                    // }

                    let prev_sep_plane = self.prev_sep_planes.get(&(rb1_idx, rb2_idx)).copied();
                    let new_sep_plane = find_separating_plane(new_rbs[rb1_idx].tmesh(), new_rbs[rb2_idx].tmesh(), prev_sep_plane);
                    if new_sep_plane.is_some() {
                        // TODO idk wtf I'm doing
                        self.prev_sep_planes.insert((rb1_idx, rb2_idx), new_sep_plane.unwrap().0);
                    } else {
                        collisions.push((rb1_idx, rb2_idx));
                    }
                }
            }

            if collisions.is_empty() {
                // Hooray - but if we were correcting a collision, let's resolve those.
                for (rb1_idx, rb2_idx) in prev_collisions {
                    let mut rb1 = new_rbs[rb1_idx].clone();
                    let mut rb2 = new_rbs[rb2_idx].clone();
                    contact_infos.push(handle_collision(&mut rb1, &mut rb2, self.prev_sep_planes[&(rb1_idx, rb2_idx)]));
                    new_rbs[rb1_idx] = rb1;
                    new_rbs[rb2_idx] = rb2;
                }

                // TODO: Add actual collision-time-find here
                latest_no_collision_t = dt;

                break;

            } else {
                prev_collisions = collisions;
                new_rbs = self.rbs.clone();
                earliest_collision_t = dt;
                dt = (earliest_collision_t + latest_no_collision_t) * 0.5;
                contact_infos.clear();
            }
        }
        // println!("dt: {}", dt);

        self.rbs = new_rbs;
        return (dt, contact_infos);
    }

    pub fn get_num_rbs(&self) -> usize {
        return self.rbs.len();
    }

    pub fn get_rb(&self, idx: usize) -> &RigidBody {
        &self.rbs[idx]
    }

    // FOR DEBUGGING
    pub fn activate_all(&mut self) {
        for rb in self.rbs.iter_mut() {
            rb.active = true;
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

#[derive(Clone, Debug)]
pub struct RigidBody {
    x: na::Vector3<Float>,  // position
    q: na::UnitQuaternion<Float>,  // rotation
    pub p: na::Vector3<Float>,  // momentum
    pub l: na::Vector3<Float>,  // angular momentum
    pub m_inv: Float, // mass
    pub i_body_inv: na::Matrix3<Float>,
    pub name: String,
    tmesh: TransformedMesh,
    // TODO FIGURE OUT HOW TO REMOVE THIS
    pub scale: na::Vector3<Float>, // dimensions (assuming it's a box)
    pub active: bool
}

impl RigidBody {
    pub fn new_box(m: Float, name: &str, x_dim: Float, y_dim: Float, z_dim: Float, mesh: &Rc<Mesh>) -> RigidBody {
        let transform = Transform {
            p: Default::default(),
            rot: na::Matrix3::identity(),
            scale: na::Vector3::new(x_dim, y_dim, z_dim)
        };
        let x2 = x_dim*x_dim;
        let y2 = y_dim*y_dim;
        let z2 = z_dim*z_dim;
        RigidBody {
            x: Default::default(),
            q: Default::default(),
            p: Default::default(),
            l: Default::default(),
            m_inv: 1.0 / m,
            i_body_inv: na::Matrix3::from_diagonal(&((12.0 / m) * na::Vector3::new(1.0 / (y2 + z2), 1.0 / (x2 + z2), 1.0 / (x2 + y2)))),
            name: String::from(name),
            tmesh: TransformedMesh::new(&Rc::clone(mesh), transform),
            scale: na::Vector3::new(x_dim, y_dim, z_dim),
            active: true
        }
    }

    pub fn new_static_box(name: &str, x_dim: Float, y_dim: Float, z_dim: Float, mesh: &Rc<Mesh>) -> RigidBody {
        let transform = Transform {
            p: Default::default(),
            rot: na::Matrix3::identity(),
            scale: na::Vector3::new(x_dim, y_dim, z_dim)
        };
        RigidBody {
            x: Default::default(),
            q: Default::default(),
            p: Default::default(),
            l: Default::default(),
            m_inv: 0.0,
            i_body_inv: na::Matrix3::zeros(),
            name: String::from(name),
            tmesh: TransformedMesh::new(&Rc::clone(mesh), transform),
            scale: na::Vector3::new(x_dim, y_dim, z_dim),
            active: true
        }
    }

    pub fn x(&self) -> na::Vector3<Float> {
        return self.x;
    }

    pub fn q(&self) -> na::UnitQuaternion<Float> {
        return self.q;
    }

    pub fn tmesh(&self) -> &TransformedMesh {
        return &self.tmesh;
    }

    // TODO: Consider caching derived values like w, rot, I_inv, etc.
    pub fn get_angular_velocity(&self) -> na::Vector3<Float> {
        return self.get_inv_inertia_tensor() * self.l;
    }

    // world-space velocity of RB's center of mass.
    pub fn get_velocity(&self) -> na::Vector3<Float> {
        return self.p * self.m_inv;
    }

    pub fn get_point_velocity(&self, p_world: &na::Vector3<Float>) -> na::Vector3<Float> {
        let r = p_world - self.x;
        let w = self.get_angular_velocity();
        let v = self.get_velocity();
        return v + w.cross(&r);
    }

    pub fn get_inv_inertia_tensor(&self) -> na::Matrix3<Float> {
        let rot = self.q.to_rotation_matrix();
        return rot * self.i_body_inv * rot.transpose();
    }

    pub fn semi_implicit_euler_update(&mut self, force: &na::Vector3<Float>, torque: &na::Vector3<Float>, dt: Float) {
        if self.m_inv == 0.0 {
            return;
        }
        let dp = force;
        self.p += dt * dp;

        let dl = torque;
        self.l += dt * dl;

        let v = self.get_velocity();
        self.x += dt * v;

        let w = self.get_angular_velocity();
        let w_as_q = na::Quaternion::new(0.0, w[0], w[1], w[2]);
        let dq = 0.5 * w_as_q * self.q.quaternion();
        self.q = na::UnitQuaternion::from_quaternion(self.q.quaternion() + dt * dq);

        self.tmesh.set_transform(Transform {
            p: self.x,
            rot: self.q.to_rotation_matrix().matrix().clone(),
            scale: self.scale
        });
    }

    pub fn explicit_euler_update(&mut self, force: &na::Vector3<Float>, torque: &na::Vector3<Float>, dt: Float) {
        if self.m_inv == 0.0 {
            return;
        }
        let v = self.get_velocity();
        let w = self.get_angular_velocity();

        let dx = v;
        let w_as_q = na::Quaternion::new(0.0, w[0], w[1], w[2]);
        let dq = 0.5 * w_as_q * self.q.quaternion();
        let dp = force;
        let dl = torque;

        self.x += dt * dx;
        self.q = na::UnitQuaternion::from_quaternion(self.q.quaternion() + dt * dq);
        self.p += dt * dp;
        self.l += dt * dl;

        self.tmesh.set_transform(Transform {
            p: self.x,
            rot: self.q.to_rotation_matrix().matrix().clone(),
            scale: self.scale
        });
    }

    pub fn get_transform(&self) -> Transform {
        return Transform {
            p: self.x,
            rot: self.q.to_rotation_matrix().matrix().clone(),
            scale: self.scale
        }
    }

    fn update_transform(&mut self) {
        self.tmesh.set_transform(Transform {
            p: self.x,
            rot: self.q.to_rotation_matrix().matrix().clone(),
            scale: self.scale
        });
    }

    pub fn set_pose(&mut self, x: &na::Vector3<Float>, q: &na::UnitQuaternion<Float>) {
        self.x = *x;
        self.q = *q;
        self.update_transform();
    }
}

fn signed_dist_from_plane(p: &na::Vector3<Float>, plane_p: &na::Vector3<Float>, plane_n: &na::Vector3<Float>) -> Float {
    return (p - plane_p).dot(plane_n);
}

fn point_on_positive_side_of_plane(p: &na::Vector3<Float>, plane_p: &na::Vector3<Float>, plane_n: &na::Vector3<Float>) -> bool {
    return signed_dist_from_plane(p, plane_p, plane_n) >= 0.0;
}

pub struct Mesh {
    pub vertices: Vec<na::Vector3<Float>>,
    pub edges: Vec<(usize, usize)>,
    pub quads: Vec<[usize; 4]>,
}

pub fn unit_box_mesh() -> Mesh {
    let mut vertices = Vec::new();
    for &x in &[-0.5, 0.5] {
        for &y in &[-0.5, 0.5] {
            for &z in &[-0.5, 0.5] {
                vertices.push(na::Vector3::new(x, y, z));
            }
        }
    }
    
    let edges = vec![
        (0, 4), (4, 5), (5, 1), (1, 0),
        (2, 3), (3, 7), (7, 6), (6, 2),
        (0, 2), (1, 3), (4, 6), (5, 7)
    ];

    let quads = vec![
        [2, 3, 7, 6],  // top
        [0, 4, 5, 1],  // bottom
        [0, 2, 6, 4], // front
        [1, 5, 7, 3], // back
        [4, 6, 7, 5], // left
        [0, 1, 3, 2], // right
    ];

    return Mesh { vertices, edges, quads};
}

#[derive(Clone)]
pub struct Transform {
    pub p: na::Vector3<Float>,
    pub rot: na::Matrix3<Float>,
    pub scale: na::Vector3<Float>
}

fn transform_point(transform: &Transform, p: &na::Vector3<Float>) -> na::Vector3<Float> {
    let mut p_scaled = *p;
    for i in 0..3 {
        p_scaled[i] *= transform.scale[i];
    }
    return transform.rot * p_scaled + transform.p;
    
}

#[derive(Clone)]
pub struct TransformedMesh {
    pub mesh: Rc<Mesh>,
    transform: Transform,
    transformed_vert_cache: RefCell<Vec<Option<na::Vector3<Float>>>>
}

impl fmt::Debug for TransformedMesh {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        return f.debug_struct("TMesh").finish();
    }
}

impl TransformedMesh {
    pub fn new(mesh: &Rc<Mesh>, transform: Transform) -> TransformedMesh {
        TransformedMesh {
            mesh: Rc::clone(mesh),
            transform,
            transformed_vert_cache: RefCell::new(vec![None; mesh.vertices.len()])
        }
    }

    // Is there a way to make this return a & and make find_separating_plane still work?
    pub fn get_vertex(&self, index: usize) -> na::Vector3<Float> {
        let v = &mut self.transformed_vert_cache.borrow_mut()[index];
        if v.is_some() {
            return v.unwrap();
        }
        *v = Some(transform_point(&self.transform, &self.mesh.vertices[index]));
        return v.unwrap();
    }

    pub fn set_transform(&mut self, transform: Transform) {
        self.transform = transform;
        for v in self.transformed_vert_cache.borrow_mut().iter_mut() {
            *v = None;
        }
    }
}

fn does_face_plane_separate_meshes<'a>(mesh1: &TransformedMesh, face_on_mesh1: &[usize; 4], mesh2: &TransformedMesh) -> Option<(Float, usize)> {
    let v0 = mesh1.get_vertex(face_on_mesh1[0]);
    let v1 = mesh1.get_vertex(face_on_mesh1[1]);
    let v2 = mesh1.get_vertex(face_on_mesh1[2]);
    let plane_n = (v1 - v0).cross(&(v2 - v0)).normalize();
    let plane_p = v0;

    let mut min_dist = Float::MAX;
    let mut min_dist_idx = 0;
    for v_idx in 0..mesh2.mesh.vertices.len() {
        let d = signed_dist_from_plane(&mesh2.get_vertex(v_idx), &plane_p, &plane_n);
        if d < 0.0 {
            // println!("FACE-VERTEX: {}", v_idx);
            return None;
        }
        if d < min_dist {
            min_dist = d;
            min_dist_idx = v_idx;
        }
    }
    return Some((min_dist, min_dist_idx));
}

// Returns:
// 0: min dist of mesh2's verts from plane
// 1: pair of dists: edge_on_mesh2's vertex dists from plane
fn does_edge_plane_separate_meshes<'a>(mesh1: &TransformedMesh, edge_on_mesh1: (usize, usize),
                                       mesh2: &TransformedMesh, edge_on_mesh2: (usize, usize)) -> Option<(Float, (Float, Float))> {
    let (m1v1, m1v2) = edge_on_mesh1;
    let (m2v1, m2v2) = edge_on_mesh2;
    let plane_p = mesh1.get_vertex(m1v1);
    let mut plane_n = (mesh1.get_vertex(m1v2) - mesh1.get_vertex(m1v1)).cross(&(mesh2.get_vertex(m2v2) - mesh2.get_vertex(m2v1))).normalize();

    let mut d = signed_dist_from_plane(&mesh2.get_vertex(0), &plane_p, &plane_n);
    if d < 0.0 {
        // We want to keep the convention of "normals point from Mesh1 to Mesh2".
        // Therefore, we change the plane normal to point in the direction of the first vertex in Mesh2.
        // If this is a valid separating plane, this will be the correct normal direction.
        //
        // TODO: This doesn't actually matter if we don't use the sign of the return value, or pass this normal
        // direction all the way to Contact creation.
        plane_n = -plane_n;
        d = -d;
    }
    let d = d;
    let plane_n = plane_n;

    let mesh2_side = d >= 0.0;
    let mut min_dist = d.abs();
    for v_idx in 1..mesh2.mesh.vertices.len() {
        let d = signed_dist_from_plane(&mesh2.get_vertex(v_idx), &plane_p, &plane_n);
        let side = d >= 0.0;
        if side != mesh2_side {
            // TODO: This debug msg is not totally helpful: this might not actually be the colliding vertex.
            // let x = mesh2.get_vertex(v_idx);
            // println!("EDGE-EDGE (rb2 vertex {} {} {} {})", v_idx, x[0], x[1], x[2]);
            return None;
        }
        min_dist = min_dist.min(d.abs());
    }
    for v_idx in 0..mesh1.mesh.vertices.len() {
        // TODO: Can we do this? I think so...
        if v_idx == m1v1 || v_idx == m1v2 {
            continue;
        }
        let d = signed_dist_from_plane(&mesh1.get_vertex(v_idx), &plane_p, &plane_n);
        let side = d >= 0.0;
        if side == mesh2_side {
            // TODO: This debug msg is not totally helpful: this might not actually be the colliding vertex.
            // let x = mesh1.get_vertex(v_idx);
            // println!("EDGE-EDGE (rb1 vertex {} {} {} {})", v_idx, x[0], x[1], x[2]);
            return None;
        }
    }
    return Some((min_dist,
        (signed_dist_from_plane(&mesh2.get_vertex(m2v1), &plane_p, &plane_n),
         signed_dist_from_plane(&mesh2.get_vertex(m2v2), &plane_p, &plane_n))));
}

#[derive(Clone, Copy)]
pub enum SeparatingPlane {
    Mesh1Face([usize; 4], usize), // face on 1 and vert on 2
    Mesh2Face([usize; 4], usize), // face on 2 and vert on 1
    Edges((usize, usize), (usize, usize))
}

// TODO: optimize the edge-edge stuff using techniques from Dirk Gregorius's GDC talk.
pub fn find_separating_plane<'a>(mesh1: &TransformedMesh, mesh2: &TransformedMesh,
                                 initial_guess: Option<SeparatingPlane>) -> Option<(SeparatingPlane, Float)> {
    if initial_guess.is_some() {
        let initial_guess = initial_guess.unwrap();
        match initial_guess {
            SeparatingPlane::Mesh1Face(f, _) => {
                let p = does_face_plane_separate_meshes(mesh1, &f, mesh2);
                if p.is_some() {
                    let p = p.unwrap();
                    return Some((SeparatingPlane::Mesh1Face(f, p.1), p.0));
                }
            },
            SeparatingPlane::Mesh2Face(f, _) => {
                let p = does_face_plane_separate_meshes(mesh2, &f, mesh1);
                if p.is_some() {
                    let p = p.unwrap();
                    return Some((SeparatingPlane::Mesh2Face(f, p.1), p.0));
                }
            },
            SeparatingPlane::Edges(e1, e2) => {
                let d = does_edge_plane_separate_meshes(mesh1, e1, mesh2, e2);
                if d.is_some() {
                    return Some((SeparatingPlane::Edges(e1, e2), d.unwrap().0))
                }
            }
        };
    }
    
    // Look for a separating plane in the faces of mesh1 and mesh2.
    // TODO: 
    for f in mesh1.mesh.quads.iter() {
        let d = does_face_plane_separate_meshes(mesh1, f, mesh2);
        if d.is_some() {
            let d = d.unwrap();
            return Some((SeparatingPlane::Mesh1Face(*f, d.1), d.0));
        }
    }
    for f in mesh2.mesh.quads.iter() {
        let d = does_face_plane_separate_meshes(mesh2, f, mesh1);
        if d.is_some() {
            let d = d.unwrap();
            return Some((SeparatingPlane::Mesh2Face(*f, d.1), d.0));
        }
    }

    // Look for a separating plane in the cross products of the edges of mesh1 and mesh2.
    let mut min_edge_dist = f32::MAX;
    let mut min_dist_edges = None;
    for &edge1 in mesh1.mesh.edges.iter() {
        for &edge2 in mesh2.mesh.edges.iter() {
            let d = does_edge_plane_separate_meshes(mesh1, edge1, mesh2, edge2);
            if d.is_some() {
                // Look for the "support edges", which are the SAT edges that are actually near each other. 
                let d = d.unwrap();
                let (ed1, ed2) = d.1;
                if ed1.abs() < min_edge_dist || ed2.abs() < min_edge_dist {
                    min_edge_dist = ed1.abs().min(ed2.abs());
                    min_dist_edges = Some((edge1, edge2));
                }
            }
        }
    }
    if min_dist_edges.is_some() {
        let min_dist_edges = min_dist_edges.unwrap();
        return Some((SeparatingPlane::Edges(min_dist_edges.0, min_dist_edges.1), min_edge_dist));
    }

    return None;
}

fn d_mnop(m: &na::Vector3<Float>, n: &na::Vector3<Float>,
             o: &na::Vector3<Float>, p: &na::Vector3<Float>) -> Float {
    return (m.x - n.x)*(o.x - p.x) + (m.y - n.y)*(o.y - p.y) + (m.z - n.z)*(o.z - p.z);
}

// http://paulbourke.net/geometry/pointlineplane/
//
// between (p1,p2) and (p3,p4)
//
// TODO: Unit test this maybe. The result looks a little off for:
// p1: -0.7071068, -0.50000006, -0.000000059604645
// p2: -0.7071068, 0.50000006, -0.000000059604645
// p3: -0.84644663, 0.14644665, -0.000000029802322
// p4: -1.3464465, -0.35355338, 0.7071068
pub fn closest_points_on_edges(p1: &na::Vector3<Float>, p2: &na::Vector3<Float>,
                               p3: &na::Vector3<Float>, p4: &na::Vector3<Float>) -> (na::Vector3<Float>, na::Vector3<Float>) {
    // TODO: What does it mean for the denominators below to be nearly zero?
    // TODO: cache and re-use repeated values here.
    let mut a = (d_mnop(p1,p3,p4,p3)*d_mnop(p4,p3,p2,p1) - d_mnop(p1,p3,p2,p1)*d_mnop(p4,p3,p4,p3)) /
        (d_mnop(p2,p1,p2,p1)*d_mnop(p4,p3,p4,p3) - d_mnop(p4,p3,p2,p1)*d_mnop(p4,p3,p2,p1));
    let mut b = (d_mnop(p1,p3,p4,p3) + a * d_mnop(p4,p3,p2,p1)) / d_mnop(p4,p3,p4,p3);

    // Should we need to do this clamp here?
    a = a.min(1.0).max(0.0);
    b = b.min(1.0).max(0.0);
    
    let point_on_edge1 = p1 + a*(p2 - p1);
    let point_on_edge2 = p3 + b*(p4 - p3);
    return (point_on_edge1, point_on_edge2);
}

pub fn closest_point_on_plane_to_point(p: &na::Vector3<Float>, plane_p: &na::Vector3<Float>, plane_n: &na::Vector3<Float>) -> na::Vector3<Float> {
    p - (p - plane_p).dot(plane_n) * plane_n
}

#[derive(Clone, Debug)]
pub struct ContactInfo {
    pub contact_p: na::Vector3<Float>,
    pub contact_n: na::Vector3<Float>,
    pub impulse: na::Vector3<Float>,
    pub v_rel_before: na::Vector3<Float>,
    pub v_rel_after: na::Vector3<Float>,
    pub rb_to: RigidBody,
    pub rb_from: RigidBody,
}

#[derive(Clone)]
pub struct UpdateInfo {
    pub sep_plane: SeparatingPlane,
    pub contact_info: Option<ContactInfo>
}

pub fn handle_collision(rb1_in: &mut RigidBody,
                        rb2_in: &mut RigidBody,
                        sep_plane: SeparatingPlane) -> ContactInfo {
    // TODO WHY DO WE NEED THIS CLONE
    let mut rb1 = rb1_in.clone();
    let mut rb2 = rb2_in.clone();
    let contact_info = {
        let (rb_to, rb_from, contact_type) = match sep_plane {
            SeparatingPlane::Mesh1Face(f, v) => {
                // println!("mesh1 face");
                (&mut rb2, &mut rb1, ContactType::VertexFace(v, f))
            },
            SeparatingPlane::Mesh2Face(f, v) => {
                // println!("mesh2 face");
                (&mut rb1, &mut rb2, ContactType::VertexFace(v, f))
            },
            SeparatingPlane::Edges(e1, e2) => {
                // println!("edges");
                (&mut rb1, &mut rb2, ContactType::EdgeEdge(e1, e2))
            }
        };
        let mut contact = make_contact(contact_type, rb_to, rb_from);

        let (v_rel_before, v_rel_after, impulse) = apply_impulses_from_contact(&mut contact);

        ContactInfo {
            contact_p: contact.position,
            contact_n: contact.normal,
            impulse,
            v_rel_before,
            v_rel_after,
            rb_to: rb_to.clone(),
            rb_from: rb_from.clone()
        }
    };

    *rb1_in = rb1;
    *rb2_in = rb2;
    return contact_info;
}

enum ContactType {
    VertexFace(usize, [usize; 4]), // rb_to's vertex, rb_from's face
    EdgeEdge((usize, usize), (usize, usize)) // (rb_to, rb_from)
}

struct Contact<'a> {
    rb_to: &'a mut RigidBody,
    rb_from: &'a mut RigidBody,
    position: na::Vector3<Float>,
    normal: na::Vector3<Float>
}

fn make_contact<'a, 'b>(contact_type: ContactType,
                        rb_to: &'a mut RigidBody, rb_from: &'a mut RigidBody) -> Contact<'a> {
    // TODO: Maybe make sure that the plane_n's are not degenerate before normalizing.
    let (position, normal) = match contact_type {
        ContactType::VertexFace(to_v, from_f) => {
            let u = rb_from.tmesh().get_vertex(from_f[1]) - rb_from.tmesh().get_vertex(from_f[0]);
            let v = rb_from.tmesh().get_vertex(from_f[2]) - rb_from.tmesh().get_vertex(from_f[0]);
            let plane_n = u.cross(&v).normalize();
            (closest_point_on_plane_to_point(&rb_to.tmesh().get_vertex(to_v), &rb_from.tmesh().get_vertex(from_f[0]), &plane_n),
                plane_n)
        },
        ContactType::EdgeEdge((to_v0, to_v1), (from_v0, from_v1)) => {
            let u = rb_to.tmesh().get_vertex(to_v1) - rb_to.tmesh().get_vertex(to_v0);
            let v = rb_from.tmesh().get_vertex(from_v1) - rb_from.tmesh().get_vertex(from_v0);
            let mut edge_n = u.cross(&v).normalize();
            if point_on_positive_side_of_plane(&rb_from.tmesh().get_vertex(from_v0), &rb_to.tmesh().get_vertex(to_v0), &edge_n) {
                edge_n = -edge_n;
            }
            (closest_points_on_edges(&rb_to.tmesh().get_vertex(to_v0), &rb_to.tmesh().get_vertex(to_v1),
                                     &rb_from.tmesh().get_vertex(from_v0), &rb_from.tmesh().get_vertex(from_v1)).1,  // arbitrarily use from's point as the contact point
                edge_n)
        }
    };
    Contact {
        rb_to, rb_from, position, normal
    }
}

// Returns: (v_rel before (to - from), v_rel_after, impulse)
fn apply_impulses_from_contact<'a>(c: &mut Contact<'a>) -> (na::Vector3<Float>, na::Vector3<Float>, na::Vector3<Float>) {
    const COEFFICIENT_OF_RESTITUTION: Float = 0.5;
    const COEFFICIENT_OF_FRICTION: Float = 0.2;
    let v_to = c.rb_to.get_point_velocity(&c.position);
    let v_from = c.rb_from.get_point_velocity(&c.position);
    let v_rel = v_to - v_from;

    let I_inv_to = c.rb_to.get_inv_inertia_tensor();
    let I_inv_from = c.rb_from.get_inv_inertia_tensor();

    let r_to = c.position - c.rb_to.x;
    let r_from = c.position - c.rb_from.x;

    // normal component
    let v_rel_n = c.normal.dot(&(v_to - v_from));
    let to_term = (I_inv_to * (r_to.cross(&c.normal))).cross(&r_to);
    let from_term = (I_inv_from * (r_from.cross(&c.normal))).cross(&r_from);
    let denom = c.rb_to.m_inv + c.rb_from.m_inv + c.normal.dot(&to_term) + c.normal.dot(&from_term);
    let j_normal = -(1.0 + COEFFICIENT_OF_RESTITUTION) * v_rel_n / denom;
    let normal_impulse = j_normal * c.normal;

    // frictional component
    let tangent = c.normal.cross(&v_rel).cross(&c.normal);
    let fric_impulse;
    // TODO come up with a better value for this I guess.
    if tangent.norm_squared() > 0.0001 * 0.0001 {
        let tangent = tangent.normalize();
        let to_term = (I_inv_to * (r_to.cross(&tangent))).cross(&r_to);
        let from_term = (I_inv_from * (r_from.cross(&tangent))).cross(&r_from);
        let denom = c.rb_to.m_inv + c.rb_from.m_inv + tangent.dot(&to_term) + tangent.dot(&from_term);
        let j_fric = (-1.0 + COEFFICIENT_OF_RESTITUTION) * v_rel.dot(&tangent) / denom;
        let j_fric = j_fric.max(-COEFFICIENT_OF_FRICTION*j_normal).min(COEFFICIENT_OF_FRICTION*j_normal);
        fric_impulse = j_fric * tangent;
    } else {
        fric_impulse = na::Vector3::zeros();
    }

    let impulse = normal_impulse + fric_impulse;

    // if c.rb_to.m_inv > 0.0 {
    //     println!("{} {} {} {}", normal_impulse.norm(), fric_impulse.norm(), c.rb_to.p.norm(), c.rb_to.l.norm());
    // }
    // if c.rb_from.m_inv > 0.0 {
    //     println!("{} {} {} {}", normal_impulse.norm(), fric_impulse.norm(), c.rb_from.p.norm(), c.rb_from.l.norm());
    // }

    c.rb_to.p += impulse;
    c.rb_from.p -= impulse;

    c.rb_to.l += r_to.cross(&impulse);
    c.rb_from.l -= r_from.cross(&impulse);

    let v_rel_new = c.rb_to.get_point_velocity(&c.position) - c.rb_from.get_point_velocity(&c.position);

    return (v_rel, v_rel_new, impulse);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn basic_separating_plane_no_collision_test() {
        let box_mesh = Rc::new(unit_box_mesh());
        let rb1 = RigidBody::new_box(1.0, "", 1.0, 1.0, 1.0, &box_mesh);

        let mut rb2 = RigidBody::new_box(1.0, "", 1.0, 1.0, 1.0, &box_mesh);
        let x = na::Vector3::new(-2.5, 0.0, 0.0);
        let q = na::UnitQuaternion::from_rotation_matrix(&na::Rotation3::from_euler_angles(na::RealField::frac_pi_4(), 0.0, na::RealField::frac_pi_4()));
        rb2.set_pose(&x, &q);

        let sep_plane = find_separating_plane(rb1.tmesh(), rb2.tmesh(), None);
        assert!(sep_plane.is_some());
        assert!(find_separating_plane(rb1.tmesh(), rb2.tmesh(), Some(sep_plane.unwrap().0)).is_some());
    }

    #[test]
    fn separating_plane_face_vertex_collision_test() {
        let box_mesh = Rc::new(unit_box_mesh());
        let rb1 = RigidBody::new_box(1.0, "", 1.0, 1.0, 1.0, &box_mesh);

        let mut rb2 = RigidBody::new_box(1.0, "", 1.0, 1.0, 1.0, &box_mesh);
        let d = 0.5 + 0.5*(2.0 as f32).sqrt() - 0.1;
        let x = na::Vector3::new(-d, 0.0, 0.0);
        let q = na::UnitQuaternion::from_rotation_matrix(&na::Rotation3::from_euler_angles(na::RealField::frac_pi_4(), 0.0, na::RealField::frac_pi_4()));
        rb2.set_pose(&x, &q);

        assert!(find_separating_plane(rb1.tmesh(), rb2.tmesh(), None).is_none());
    }

    #[test]
    fn separating_plane_edge_face_collision_test() {
        let box_mesh = Rc::new(unit_box_mesh());
        let mut rb1 = RigidBody::new_box(1.0, "", 1.0, 1.0, 1.0, &box_mesh);
        let q = na::UnitQuaternion::from_rotation_matrix(&na::Rotation3::from_euler_angles(0.0, na::RealField::frac_pi_4(), 0.0));
        rb1.set_pose(&rb1.x(), &q);

        let mut rb2 = RigidBody::new_box(1.0, "", 1.0, 1.0, 1.0, &box_mesh);
        let d : f32 = (2.0 as f32).sqrt() - 0.1;
        let x = na::Vector3::new(-d, 0.0, 0.0);
        let q = na::UnitQuaternion::from_rotation_matrix(&na::Rotation3::from_euler_angles(0.0, 0.0, na::RealField::frac_pi_4()));
        rb2.set_pose(&x, &q);

        assert!(find_separating_plane(rb1.tmesh(), rb2.tmesh(), None).is_none());
    }

    #[test]
    fn separating_plane_no_collision_requires_edge_edge() {
        let box_mesh = Rc::new(unit_box_mesh());
        let mut rb1 = RigidBody::new_box(1.0, "", 1.0, 1.0, 1.0, &box_mesh);
        let q = na::UnitQuaternion::from_rotation_matrix(&na::Rotation3::from_euler_angles(0.0, na::RealField::frac_pi_4(), 0.0));
        rb1.set_pose(&rb1.x(), &q);

        let mut rb2 = RigidBody::new_box(1.0, "", 1.0, 1.0, 1.0, &box_mesh);
        let x = na::Vector3::new(-1.7, 0.0, 0.0);
        let q = na::UnitQuaternion::from_euler_angles(0.7938401, -1.3559403, -2.985731);
        rb2.set_pose(&x, &q);

        assert!(find_separating_plane(&rb1.tmesh(), &rb2.tmesh(), None).is_some());
    }

    #[test]
    fn separating_plane_maybe_degenerate_collision_test() {
        let box_mesh = Rc::new(unit_box_mesh());
        let rb1 = RigidBody::new_box(1.0, "", 1.0, 1.0, 1.0, &box_mesh);

        let mut rb2 = RigidBody::new_box(1.0, "", 1.0, 1.0, 1.0, &box_mesh);
        rb2.set_pose(&na::Vector3::new(-0.9, 0.0, 0.0), &rb2.q());

        assert!(find_separating_plane(&rb1.tmesh(), &rb2.tmesh(), None).is_none());
    }

    // YO This one is really hard! It's not gravity that's causing this oscillation.
    #[test]
    fn resting_edge_edge_test() {
        let mut ps = Scene::new();
        ps.enable_gravity = false;
        let box_mesh = Rc::new(unit_box_mesh());
        // wall
        // TODO: wall has non-zero p and l. Does that matter?
        // let mut wall = RigidBody::new_static_box("wall", 10.0, 1.0, 10.0, &box_mesh);
        // let x = na::Vector3::new(0.0, -5.5, 0.0);
        // let q: na::UnitQuaternion<Float> = Default::default();
        // wall.set_pose(&x, &q);
        // let wall = ps.add_rb(wall);
        // box1
        let mut box1 = RigidBody::new_box(1.0, "box1", 1.0, 1.0, 1.0, &box_mesh);
        let x = na::Vector3::new(-3.0169132, -3.3653288, -3.5509505);
        let q = na::UnitQuaternion::<Float>::from_euler_angles(-0.8039882, 0.28533742, -2.1953382);
        box1.set_pose(&x, &q);
        box1.p = na::Vector3::new(-0.9383354, 0.45622417, 0.519677);
        box1.l = na::Vector3::new(0.9216851, 0.91954553, -0.378117);
        box1.active = false;
        let box1 = ps.add_rb(box1);
        // box2
        let mut box2 = RigidBody::new_box(1.0, "box2", 1.0, 1.0, 1.0, &box_mesh);
        let x = na::Vector3::new(-2.859328, -4.4902024, -2.6403682);
        let q = na::UnitQuaternion::<Float>::from_euler_angles(-0.018460046, -0.21316701, 0.0016995296);
        box2.set_pose(&x, &q);
        box2.p = na::Vector3::new(-0.229679, -0.25147414, 0.82076985);
        box2.l = na::Vector3::new(0.09784485, 0.049024202, 0.040893458);
        box2.active = false;
        let box2 = ps.add_rb(box2);

        let dt = 1.0 / 60.0;
        let (dt_out, contact_infos) = ps.update(dt);
        println!("warmup dt: {}, warmup contacts: {}", dt_out, contact_infos.len());
        ps.activate_all();
        let (dt_out, contact_infos) = ps.update(dt);
        println!("dt: {}", dt_out);
        for c in contact_infos {
            println!("v_rel facts: {} {}", c.v_rel_before.dot(&c.contact_n), c.v_rel_after.dot(&c.contact_n));
            println!("{:#?}", c);
            println!("===============");
        }
        println!("ITERATIONNNNNNNNNNNNNNNNNNNNNNNNNN");
        let (dt_out, contact_infos) = ps.update(dt);
        println!("dt: {}", dt_out);
        for c in contact_infos {
            println!("v_rel facts: {} {}", c.v_rel_before.dot(&c.contact_n), c.v_rel_after.dot(&c.contact_n));
            println!("{:#?}", c);
            println!("===============");
        }

        println!("ITERATIONNNNNNNNNNNNNNNNNNNNNNNNNN");
        let (dt_out, contact_infos) = ps.update(dt);
        println!("dt: {}", dt_out);
        for c in contact_infos {
            println!("v_rel facts: {} {}", c.v_rel_before.dot(&c.contact_n), c.v_rel_after.dot(&c.contact_n));
            println!("{:#?}", c);
            println!("===============");
        }
    }

    #[test]
    fn resting_vertex_face_test() {
        let mut ps = Scene::new();
        ps.enable_gravity = true;
        let box_mesh = Rc::new(unit_box_mesh());
        // wall
        let mut wall = RigidBody::new_static_box("wall", 10.0, 1.0, 10.0, &box_mesh);
        let x = na::Vector3::new(0.0, -5.5, 0.0);
        let q: na::UnitQuaternion<Float> = Default::default();
        wall.set_pose(&x, &q);
        wall.p = na::Vector3::new(-0.02534554, -0.44694892, -0.015009386);
        wall.l = na::Vector3::new(-1.3706628, 0.027466798, 1.4966589);
        let wall = ps.add_rb(wall);
        // box1
        let mut box1 = RigidBody::new_box(1.0, "box1", 1.0, 1.0, 1.0, &box_mesh);
        let x = na::Vector3::new(-2.9542038, -4.258529, -2.8331451);
        let q = na::UnitQuaternion::<Float>::from_euler_angles(-0.53423434, -0.106729515, 0.13080074);
        box1.set_pose(&x, &q);
        box1.p = na::Vector3::new(0.02534554, -0.45102382, 0.015009386);
        box1.l = na::Vector3::new(0.085633464, 0.00000000014370016, -0.14460458);
        box1.active = false;
        let box1 = ps.add_rb(box1);

        let dt = 1.0 / 60.0;
        let (dt_out, contact_infos) = ps.update(dt);
        println!("warmup dt: {}, warmup contacts: {}", dt_out, contact_infos.len());
        ps.activate_all();

        let (dt_out, contact_infos) = ps.update(dt);
        println!("dt: {}", dt_out);
        for c in contact_infos {
            println!("v_rel facts: {} {}", c.v_rel_before.dot(&c.contact_n), c.v_rel_after.dot(&c.contact_n));
            println!("{:#?}", c);
            println!("===============");
        }

        println!("ITERATIONNNNNNNNNNNNNNNNNNNNNNNNNN");
        let (dt_out, contact_infos) = ps.update(dt);
        println!("dt: {}", dt_out);
        for c in contact_infos {
            println!("v_rel facts: {} {}", c.v_rel_before.dot(&c.contact_n), c.v_rel_after.dot(&c.contact_n));
            println!("{:#?}", c);
            println!("===============");
        }
    }
}