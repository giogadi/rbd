use nalgebra as na;

pub type Float = f32;

#[derive(Default, Clone)]
pub struct RigidBody {
    pub x: na::Vector3<Float>,  // position
    pub q: na::UnitQuaternion<Float>,  // rotation
    pub p: na::Vector3<Float>,  // momentum
    pub l: na::Vector3<Float>,  // angular momentum
    pub m_inv: Float, // mass
    pub i_body_inv: na::Matrix3<Float>,
    pub name: String,
    // TODO FIGURE OUT HOW TO REMOVE THIS
    pub scale: na::Vector3<Float> // dimensions (assuming it's a box)
}

impl RigidBody {
    pub fn new_box(m: Float, name: &str, x_dim: Float, y_dim: Float, z_dim: Float) -> RigidBody {
        let x2 = x_dim*x_dim;
        let y2 = y_dim*y_dim;
        let z2 = z_dim*z_dim;
        RigidBody {
            m_inv: 1.0 / m,
            i_body_inv: na::Matrix3::from_diagonal(&((12.0 / m) * na::Vector3::new(1.0 / (y2 + z2), 1.0 / (x2 + z2), 1.0 / (x2 + y2)))),
            scale: na::Vector3::new(x_dim, y_dim, z_dim),
            name: String::from(name),
            ..Default::default()
        }
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

    pub fn update(&mut self, force: &na::Vector3<Float>, torque: &na::Vector3<Float>, dt: Float) {
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
    }

    pub fn get_transform(&self) -> Transform {
        return Transform {
            p: self.x,
            rot: self.q.to_rotation_matrix().matrix().clone(),
            scale: self.scale
        }
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

pub struct TransformedMesh<'a> {
    pub mesh: &'a Mesh,
    transform: Transform,
    transformed_vert_cache: Vec<Option<na::Vector3<Float>>>,

}

// TODO try to follow interior mutability pattern
impl<'a> TransformedMesh<'a> {
    pub fn new(mesh: &'a Mesh, transform: Transform) -> TransformedMesh<'a> {
        TransformedMesh {
            mesh,
            transform,
            transformed_vert_cache: vec![None; mesh.vertices.len()]
        }
    }

    // Is there a way to make this return a & and make find_separating_plane still work?
    pub fn get_vertex(&mut self, index: usize) -> na::Vector3<Float> {
        let v = &mut self.transformed_vert_cache[index];
        if v.is_some() {
            return v.unwrap();
        }
        *v = Some(transform_point(&self.transform, &self.mesh.vertices[index]));
        return v.unwrap();
    }

    pub fn set_transform(&mut self, transform: Transform) {
        self.transform = transform;
        for v in self.transformed_vert_cache.iter_mut() {
            *v = None;
        }
    }
}

fn does_face_plane_separate_meshes<'a>(mesh1: &mut TransformedMesh<'a>, face_on_mesh1: &[usize; 4], mesh2: &mut TransformedMesh<'a>) -> Option<(Float, usize)> {
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
fn does_edge_plane_separate_meshes<'a>(mesh1: &mut TransformedMesh<'a>, edge_on_mesh1: (usize, usize),
                                       mesh2: &mut TransformedMesh<'a>, edge_on_mesh2: (usize, usize)) -> Option<(Float, (Float, Float))> {
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

pub fn find_separating_plane<'a>(mesh1: &mut TransformedMesh<'a>, mesh2: &mut TransformedMesh<'a>,
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

#[derive(Clone, Copy)]
pub struct ContactInfo {
    pub contact_p: na::Vector3<Float>,
    pub contact_n: na::Vector3<Float>,
    pub impulse: na::Vector3<Float>
}

#[derive(Clone, Copy)]
pub struct UpdateInfo {
    pub sep_plane: SeparatingPlane,
    pub contact_info: Option<ContactInfo>
}

pub fn update<'a>(rb1_in: &mut RigidBody, tmesh1: &mut TransformedMesh<'a>,
                  rb2_in: &mut RigidBody, tmesh2: &mut TransformedMesh<'a>,
                  mut prev_sep_plane: Option<SeparatingPlane>,
                  dt_in: Float) -> UpdateInfo {
    let mut rb1 = rb1_in.clone();
    rb1.update(&na::Vector3::new(0.0, 0.0, 0.0), &na::Vector3::new(0.0, 0.0, 0.0), dt_in);
    tmesh1.set_transform(rb1.get_transform());

    let mut rb2 = rb2_in.clone();
    rb2.update(&na::Vector3::new(0.0, 0.0, 0.0), &na::Vector3::new(0.0, 0.0, 0.0), dt_in);
    tmesh2.set_transform(rb2.get_transform());
    
    let mut new_sep_plane = find_separating_plane(tmesh1, tmesh2, prev_sep_plane);
    
    // TODO: Here, if we see a separating plane FIRST, we early-exit. However, the shapes might already be within the threshold
    // of collision we look for below. Should we check for that?
    if new_sep_plane.is_some() {
        *rb1_in = rb1;
        *rb2_in = rb2;
        return UpdateInfo { sep_plane: new_sep_plane.unwrap().0, contact_info: None };
    }

    // WE FOUND A COLLISION. Let's find the time of collision!
    let mut latest_no_collision_t = 0.0;
    let mut earliest_collision_t = dt_in;
    let mut dt = (earliest_collision_t + latest_no_collision_t) * 0.5;
    let mut n = 0;
    loop {
        n = n + 1;
        // QUESTION: when simulating forward from a collision-free state, should we play from t=0 or starting from that later state?
        rb1 = rb1_in.clone();
        tmesh1.set_transform(rb1.get_transform());
        rb1.update(&na::Vector3::new(0.0, 0.0, 0.0), &na::Vector3::new(0.0, 0.0, 0.0), dt);
        tmesh1.set_transform(rb1.get_transform());

        rb2 = rb2_in.clone();
        tmesh2.set_transform(rb2.get_transform());
        rb2.update(&na::Vector3::new(0.0, 0.0, 0.0), &na::Vector3::new(0.0, 0.0, 0.0), dt);
        tmesh2.set_transform(rb2.get_transform());
        
        new_sep_plane = find_separating_plane(tmesh1, tmesh2, prev_sep_plane);
        if new_sep_plane.is_some() {
            let (sep_plane, d) = new_sep_plane.unwrap();
            println!("ITERATING: {}", d.abs());
            // const TOLERANCE: f32 = 0.0001; DO NOT SUBMIT
            const TOLERANCE: f32 = 100.0;
            if d.abs() < TOLERANCE {
                break;
            }
            
            prev_sep_plane = Some(sep_plane);
            latest_no_collision_t = dt;
        } else {
            earliest_collision_t = dt;
        }
        dt = (earliest_collision_t + latest_no_collision_t) * 0.5;
    }

    // println!("NUM ITERATIONS: {}", n);

    // rb1_in.p = na::Vector3::new(0.0, 0.0, 0.0);
    // rb1_in.l = na::Vector3::new(0.0, 0.0, 0.0);
    // rb2_in.p = na::Vector3::new(0.0, 0.0, 0.0);
    // rb2_in.l = na::Vector3::new(0.0, 0.0, 0.0);

    let contact_info = {
        let sep_plane = new_sep_plane.unwrap().0;
        let (rb_to, rb_from, tmesh_to, tmesh_from, contact_type) = match sep_plane {
            SeparatingPlane::Mesh1Face(f, v) => {
                println!("mesh1 face");
                (&mut rb2, &mut rb1, tmesh2, tmesh1, ContactType::VertexFace(v, f))
            },
            SeparatingPlane::Mesh2Face(f, v) => {
                println!("mesh2 face");
                (&mut rb1, &mut rb2, tmesh1, tmesh2, ContactType::VertexFace(v, f))
            },
            SeparatingPlane::Edges(e1, e2) => {
                println!("edges");
                (&mut rb1, &mut rb2, tmesh1, tmesh2, ContactType::EdgeEdge(e1, e2))
            }
        };
        let mut contact = make_contact(contact_type, rb_to, rb_from, tmesh_to, tmesh_from);

        let impulse = apply_impulses_from_contact(&mut contact);

        ContactInfo {
            contact_p: contact.position,
            contact_n: contact.normal,
            impulse
        }
    };

    *rb1_in = rb1;
    *rb2_in = rb2;

    return UpdateInfo {
        sep_plane: new_sep_plane.unwrap().0,
        contact_info: Some(contact_info)
    };
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
                        rb_to: &'a mut RigidBody, rb_from: &'a mut RigidBody,
                        tmesh_to: &'a mut TransformedMesh<'b>, tmesh_from: &'a mut TransformedMesh<'b>) -> Contact<'a> {
    let (position, normal) = match contact_type {
        ContactType::VertexFace(to_v, from_f) => {
            let u = tmesh_from.get_vertex(from_f[1]) - tmesh_from.get_vertex(from_f[0]);
            let v = tmesh_from.get_vertex(from_f[2]) - tmesh_from.get_vertex(from_f[0]);
            let plane_n = u.cross(&v).normalize();
            (closest_point_on_plane_to_point(&tmesh_to.get_vertex(to_v), &tmesh_from.get_vertex(from_f[0]), &plane_n),
                plane_n)
        },
        ContactType::EdgeEdge((to_v0, to_v1), (from_v0, from_v1)) => {
            let u = tmesh_to.get_vertex(to_v1) - tmesh_to.get_vertex(to_v0);
            let v = tmesh_from.get_vertex(from_v1) - tmesh_from.get_vertex(from_v0);
            let mut edge_n = u.cross(&v).normalize();
            if point_on_positive_side_of_plane(&tmesh_from.get_vertex(from_v0), &tmesh_to.get_vertex(to_v0), &edge_n) {
                edge_n = -edge_n;
            }
            (closest_points_on_edges(&tmesh_to.get_vertex(to_v0), &tmesh_to.get_vertex(to_v1),
                                        &tmesh_from.get_vertex(from_v0), &tmesh_from.get_vertex(from_v1)).1,  // arbitrarily use from's point as the contact point
                edge_n)
        }
    };
    Contact {
        rb_to, rb_from, position, normal
    }
}

fn apply_impulses_from_contact<'a>(c: &mut Contact<'a>) -> na::Vector3<Float> {
    const COEFFICIENT_OF_RESTITUTION: Float = 0.5;
    let v_to = c.rb_to.get_point_velocity(&c.position);
    let v_from = c.rb_from.get_point_velocity(&c.position);
    let v_rel = c.normal.dot(&(v_to - v_from));

    let I_inv_to = c.rb_to.get_inv_inertia_tensor();
    let I_inv_from = c.rb_from.get_inv_inertia_tensor();

    let r_to = c.position - c.rb_to.x;
    let r_from = c.position - c.rb_from.x;

    let to_term = (I_inv_to * (r_to.cross(&c.normal))).cross(&r_to);
    let from_term = (I_inv_from * (r_from.cross(&c.normal))).cross(&r_from);
    let denom = c.rb_to.m_inv + c.rb_from.m_inv + c.normal.dot(&to_term) + c.normal.dot(&from_term);
    let j = -(1.0 + COEFFICIENT_OF_RESTITUTION) * v_rel / denom;
    let impulse = j * c.normal;

    println!("impulse: {}", impulse);
    println!("from {}, to {}", c.rb_from.name, c.rb_to.name);

    c.rb_to.p += impulse;
    c.rb_from.p -= impulse;

    c.rb_to.l += r_to.cross(&impulse);
    c.rb_from.l -= r_from.cross(&impulse);

    return impulse;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn basic_separating_plane_no_collision_test() {
        let box_mesh = unit_box_mesh();
        let rb1 = RigidBody::new_box(1.0, "", 1.0, 1.0, 1.0);
        let mut tmesh1 = TransformedMesh::new(&box_mesh, rb1.get_transform());

        let mut rb2 = RigidBody::new_box(1.0, "", 1.0, 1.0, 1.0);
        rb2.x = na::Vector3::new(-2.5, 0.0, 0.0);
        rb2.q = na::UnitQuaternion::from_rotation_matrix(&na::Rotation3::from_euler_angles(na::RealField::frac_pi_4(), 0.0, na::RealField::frac_pi_4()));
        let mut tmesh2 = TransformedMesh::new(&box_mesh, rb2.get_transform());

        let sep_plane = find_separating_plane(&mut tmesh1, &mut tmesh2, None);
        assert!(sep_plane.is_some());
        assert!(find_separating_plane(&mut tmesh1, &mut tmesh2, Some(sep_plane.unwrap().0)).is_some());
    }

    #[test]
    fn separating_plane_face_vertex_collision_test() {
        let box_mesh = unit_box_mesh();
        let rb1 = RigidBody::new_box(1.0, "", 1.0, 1.0, 1.0);
        let mut tmesh1 = TransformedMesh::new(&box_mesh, rb1.get_transform());

        let mut rb2 = RigidBody::new_box(1.0, "", 1.0, 1.0, 1.0);
        let d = 0.5 + 0.5*(2.0 as f32).sqrt() - 0.1;
        rb2.x = na::Vector3::new(-d, 0.0, 0.0);
        rb2.q = na::UnitQuaternion::from_rotation_matrix(&na::Rotation3::from_euler_angles(na::RealField::frac_pi_4(), 0.0, na::RealField::frac_pi_4()));
        let mut tmesh2 = TransformedMesh::new(&box_mesh, rb2.get_transform());

        assert!(find_separating_plane(&mut tmesh1, &mut tmesh2, None).is_none());
    }

    #[test]
    fn separating_plane_edge_face_collision_test() {
        let box_mesh = unit_box_mesh();
        let mut rb1 = RigidBody::new_box(1.0, "", 1.0, 1.0, 1.0);
        rb1.q = na::UnitQuaternion::from_rotation_matrix(&na::Rotation3::from_euler_angles(0.0, na::RealField::frac_pi_4(), 0.0));
        let mut tmesh1 = TransformedMesh::new(&box_mesh, rb1.get_transform());

        let mut rb2 = RigidBody::new_box(1.0, "", 1.0, 1.0, 1.0);
        let d : f32 = (2.0 as f32).sqrt() - 0.1;
        rb2.x = na::Vector3::new(-d, 0.0, 0.0);
        rb2.q = na::UnitQuaternion::from_rotation_matrix(&na::Rotation3::from_euler_angles(0.0, 0.0, na::RealField::frac_pi_4()));
        let mut tmesh2 = TransformedMesh::new(&box_mesh, rb1.get_transform());

        assert!(find_separating_plane(&mut tmesh1, &mut tmesh2, None).is_none());
    }

    #[test]
    fn separating_plane_no_collision_requires_edge_edge() {
        let box_mesh = unit_box_mesh();
        let mut rb1 = RigidBody::new_box(1.0, "", 1.0, 1.0, 1.0);
        rb1.q = na::UnitQuaternion::from_rotation_matrix(&na::Rotation3::from_euler_angles(0.0, na::RealField::frac_pi_4(), 0.0));
        let mut tmesh1 = TransformedMesh::new(&box_mesh, rb1.get_transform());

        let mut rb2 = RigidBody::new_box(1.0, "", 1.0, 1.0, 1.0);
        rb2.x = na::Vector3::new(-1.7, 0.0, 0.0);
        rb2.q = na::UnitQuaternion::from_euler_angles(0.7938401, -1.3559403, -2.985731);
        let mut tmesh2 = TransformedMesh::new(&box_mesh, rb2.get_transform());

        assert!(find_separating_plane(&mut tmesh1, &mut tmesh2, None).is_some());
    }

    #[test]
    fn  separating_plane_maybe_degenerate_collision_test() {
        let box_mesh = unit_box_mesh();
        let rb1 = RigidBody::new_box(1.0, "", 1.0, 1.0, 1.0);
        let mut tmesh1 = TransformedMesh::new(&box_mesh, rb1.get_transform());

        let mut rb2 = RigidBody::new_box(1.0, "", 1.0, 1.0, 1.0);
        rb2.x = na::Vector3::new(-0.9, 0.0, 0.0);
        let mut tmesh2 = TransformedMesh::new(&box_mesh, rb2.get_transform());

        assert!(find_separating_plane(&mut tmesh1, &mut tmesh2, None).is_none());
    }
}