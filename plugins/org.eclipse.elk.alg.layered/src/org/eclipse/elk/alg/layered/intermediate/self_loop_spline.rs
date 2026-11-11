// SPDX-License-Identifier: EPL-2.0
// Research modification to openedges/elk-rs; see the accompanying patch provenance.
use crate::org::eclipse::elk::alg::layered::p5edges::splines::{NubSpline, SplinesMath};
use org_eclipse_elk_core::org::eclipse::elk::core::math::{
    kvector::KVector, kvector_chain::KVectorChain,
};
use org_eclipse_elk_core::org::eclipse::elk::core::options::port_side::PortSide;

pub(super) fn spline_control_points(
    source_anchor: KVector,
    target_anchor: KVector,
    bend_points: &[KVector],
    source_side: PortSide,
    clockwise: bool,
    edge_label_distance: f64,
    node_size: KVector,
) -> Vec<KVector> {
    let mut spline_points = KVectorChain::new();
    spline_points.add_vector(source_anchor);

    if bend_points.len() >= 2 {
        let mut curr_side = source_side;

        for (index, window) in bend_points.windows(2).enumerate() {
            let first_bp = &window[0];
            let second_bp = &window[1];

            spline_points.add_vector(*first_bp);
            let length =
                ((second_bp.x - first_bp.x).powi(2) + (second_bp.y - first_bp.y).powi(2)).sqrt();
            // Uniform cubic spans can blend across a long side and cut into the owner.
            // Near-corner supports keep each span in a common exterior half-plane.
            // Reserve half the corner gap so supports do not touch the owner boundary.
            if index > 0 && length > 0.0 {
                let previous_side = if clockwise {
                    curr_side.left()
                } else {
                    curr_side.right()
                };
                let cut =
                    (exterior_gap(*first_bp, previous_side, node_size) * 0.5).min(length * 0.25);
                spline_points.add_vector(along(*first_bp, *second_bp, cut / length));
            }

            // Compute midpoint offset away from node
            let direction = SplinesMath::port_side_to_direction(curr_side);
            let mid = KVector::with_values(
                (first_bp.x + second_bp.x) * 0.5 + direction.cos() * edge_label_distance,
                (first_bp.y + second_bp.y) * 0.5 + direction.sin() * edge_label_distance,
            );
            spline_points.add_vector(mid);
            if index + 2 < bend_points.len() && length > 0.0 {
                let next_side = if clockwise {
                    curr_side.right()
                } else {
                    curr_side.left()
                };
                let cut = (exterior_gap(*second_bp, next_side, node_size) * 0.5).min(length * 0.25);
                spline_points.add_vector(along(*second_bp, *first_bp, cut / length));
            }

            curr_side = if clockwise {
                curr_side.right()
            } else {
                curr_side.left()
            };
        }

        spline_points.add_vector(*bend_points.last().unwrap());
    }

    spline_points.add_vector(target_anchor);

    let mut nub = NubSpline::new(true, 3, spline_points);
    nub.get_bezier_cp_default().to_array()
}

fn exterior_gap(point: KVector, side: PortSide, size: KVector) -> f64 {
    match side {
        PortSide::North => -point.y,
        PortSide::South => point.y - size.y,
        PortSide::West => -point.x,
        PortSide::East => point.x - size.x,
        _ => 0.0,
    }
    .max(0.0)
}

fn along(a: KVector, b: KVector, fraction: f64) -> KVector {
    KVector::with_values(a.x + (b.x - a.x) * fraction, a.y + (b.y - a.y) * fraction)
}

#[cfg(test)]
mod spline_owner_tests {
    use super::*;

    #[test]
    fn wide_two_side_loop_has_exterior_bezier_control_hulls() {
        let mut points = vec![KVector::with_values(300.0, 40.0)];
        points.extend(spline_control_points(
            KVector::with_values(300.0, 40.0),
            KVector::with_values(0.0, 80.0),
            &[
                KVector::with_values(310.0, 40.0),
                KVector::with_values(310.0, 110.0),
                KVector::with_values(-10.0, 110.0),
                KVector::with_values(-10.0, 80.0),
            ],
            PortSide::East,
            true,
            2.0,
            KVector::with_values(300.0, 100.0),
        ));
        points.push(KVector::with_values(0.0, 80.0));
        for joint in (3..points.len() - 1).step_by(3) {
            let a = points[joint];
            let incoming = (a.x - points[joint - 1].x, a.y - points[joint - 1].y);
            let outgoing = (points[joint + 1].x - a.x, points[joint + 1].y - a.y);
            assert!((incoming.0 * outgoing.1 - incoming.1 * outgoing.0).abs() < 1e-6);
            assert!(incoming.0 * outgoing.0 + incoming.1 * outgoing.1 > 0.0);
        }
        assert_eq!((points.len() - 1) % 3, 0);
        let span_count = (points.len() - 1) / 3;
        for (i, controls) in points.windows(4).step_by(3).enumerate() {
            if i > 0 && i + 1 < span_count {
                assert!(
                    controls.iter().all(|p| p.x >= 305.0 - 1e-9)
                        || controls.iter().all(|p| p.x <= -5.0 + 1e-9)
                        || controls.iter().all(|p| p.y >= 105.0 - 1e-9)
                        || controls.iter().all(|p| p.y <= -5.0 + 1e-9),
                    "non-exit span {i} loses the reserved outside gap"
                );
            }
            assert!(
                controls.iter().all(|p| p.x >= 300.0 - 1e-9)
                    || controls.iter().all(|p| p.x <= 1e-9)
                    || controls.iter().all(|p| p.y >= 100.0 - 1e-9)
                    || controls.iter().all(|p| p.y <= 1e-9),
                "span {i} has no common exterior half-plane: {controls:?}"
            );
        }
    }
    #[test]
    fn rotations_and_reversed_loops_keep_exterior_hulls() {
        let original = [
            KVector::with_values(1000.0, 8.0),
            KVector::with_values(1010.0, 8.0),
            KVector::with_values(1010.0, 57.0),
            KVector::with_values(-25.0, 57.0),
            KVector::with_values(-25.0, 32.0),
            KVector::with_values(0.0, 32.0),
        ];
        for turns in 0..4 {
            let mut points = original.to_vec();
            let mut size = KVector::with_values(1000.0, 40.0);
            let mut source_side = PortSide::East;
            for _ in 0..turns {
                points = points
                    .into_iter()
                    .map(|p| KVector::with_values(size.y - p.y, p.x))
                    .collect();
                size = KVector::with_values(size.y, size.x);
                source_side = source_side.right();
            }
            for reverse in [false, true] {
                let mut route = points.clone();
                let side = if reverse {
                    source_side.right().right()
                } else {
                    source_side
                };
                if reverse {
                    route.reverse();
                }
                let mut bezier = vec![route[0]];
                bezier.extend(spline_control_points(
                    route[0],
                    route[5],
                    &route[1..5],
                    side,
                    !reverse,
                    2.0,
                    size,
                ));
                bezier.push(route[5]);
                for controls in bezier.windows(4).step_by(3) {
                    assert!(
                        controls.iter().all(|p| p.x >= size.x - 1e-9)
                            || controls.iter().all(|p| p.x <= 1e-9)
                            || controls.iter().all(|p| p.y >= size.y - 1e-9)
                            || controls.iter().all(|p| p.y <= 1e-9)
                    );
                }
            }
        }
    }
}
