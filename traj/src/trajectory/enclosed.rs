//! Outward evaluation of the complete non-polynomial metric expression.
//!
//! The stored dense coefficients and rotation vectors are the exact inputs to
//! this graph. No point samples or empirical derivative constants establish
//! inclusion. All values and derivatives use interval operations, including
//! the two noncommuting rotations in an IMU-conditioned orientation bridge.

use super::dense::DenseSegment;
use super::roots::{OutwardInterval, ScalarEnclosure};
use crate::enclosure::{
    EnclosedJet2, EnclosureError, NativeEnclosureV1 as I, ellipsoid_up_jet,
    rodrigues_rotate_enclosed,
};
use crate::frame::ReferenceEllipsoid;
use crate::metric::{MetricError, SpeedQuantity};

type V = [I; 3];
type J = EnclosedJet2<f64>;

pub(super) struct RootExpression {
    coefficient: [[f64; 4]; 3],
    rotation: [f64; 3],
    correction: [f64; 3],
    quaternion: [f64; 4],
    duration: f64,
    lever: [f64; 3],
}

impl RootExpression {
    pub(super) fn new(segment: &DenseSegment, lever: [f64; 3]) -> Result<Self, MetricError> {
        let bridge = segment
            .derived_orientation_bridge()
            .map_err(|_| MetricError::NumericalFailure)?;
        Ok(Self {
            coefficient: segment.position_coefficient,
            rotation: bridge.integrated_rotation_body,
            correction: bridge.endpoint_correction_body,
            quaternion: segment
                .start
                .orientation_ecef_from_body
                .quaternion()
                .components_wxyz(),
            duration: segment.duration_seconds,
            lever,
        })
    }

    pub(super) fn gate(
        &self,
        lower: f64,
        upper: f64,
        center: [f64; 3],
        normal: [f64; 3],
    ) -> Result<ScalarEnclosure, MetricError> {
        self.enclose(
            lower,
            upper,
            |s| {
                let normal = vector(normal)?;
                // Subtract the anchor before adding the small lever displacement.
                // This preserves sub-millimetre roots at terrestrial ECEF scales.
                let position = add(
                    self.translation(s, 0, center)?,
                    self.rotated(s, self.lever, 0)?,
                )?;
                let first = self.point(s, 1)?;
                let second = self.point(s, 2)?;
                Ok(J {
                    value: I::dot(position, normal)?,
                    first: I::dot(first, normal)?,
                    second: I::dot(second, normal)?,
                })
            },
            false,
        )
    }

    pub(super) fn speed(
        &self,
        lower: f64,
        upper: f64,
        ellipsoid: ReferenceEllipsoid,
        quantity: SpeedQuantity,
        target: Option<f64>,
    ) -> Result<ScalarEnclosure, MetricError> {
        if target.is_some_and(|value| {
            !value.is_finite() || (value < 0.0 && quantity != SpeedQuantity::BodyLongitudinalSigned)
        }) {
            return Err(MetricError::InvalidDefinition);
        }
        self.enclose(
            lower,
            upper,
            |s| {
                let inverse_duration = I::one().div(I::point_f64(self.duration)?)?;
                let velocity = scale(self.point(s, 1)?, inverse_duration)?;
                let first = scale(self.point(s, 2)?, inverse_duration)?;
                let second = scale(self.point(s, 3)?, inverse_duration)?;
                let velocity = jets(velocity, first, second);
                let mut speed = match quantity {
                    SpeedQuantity::Spatial3d => dot_jets(velocity, velocity)?,
                    SpeedQuantity::InstantaneousHorizontal => {
                        let point = jets(self.point(s, 0)?, self.point(s, 1)?, self.point(s, 2)?);
                        let up = ellipsoid_up_jet(
                            point,
                            ellipsoid.semi_major_axis_m(),
                            ellipsoid.inverse_flattening(),
                        )?;
                        let vertical = dot_jets(up, velocity)?;
                        dot_jets(velocity, velocity)?.sub(vertical.square()?)?
                    }
                    SpeedQuantity::BodyLongitudinalSigned
                    | SpeedQuantity::BodyLongitudinalMagnitude => {
                        let axis = [1.0, 0.0, 0.0];
                        let longitudinal = jets(
                            self.rotated(s, axis, 0)?,
                            self.rotated(s, axis, 1)?,
                            self.rotated(s, axis, 2)?,
                        );
                        let body = dot_jets(longitudinal, velocity)?;
                        if target.is_some() && quantity == SpeedQuantity::BodyLongitudinalMagnitude
                        {
                            body.square()?
                        } else {
                            body
                        }
                    }
                };
                if let Some(target) = target {
                    let target = I::point_f64(target)?;
                    let target = if quantity == SpeedQuantity::BodyLongitudinalSigned {
                        target
                    } else {
                        target.square()?
                    };
                    speed.value = speed.value.sub(target)?;
                }
                Ok(speed)
            },
            target.is_none(),
        )
    }

    fn enclose(
        &self,
        lower: f64,
        upper: f64,
        equation: impl Fn(I) -> Result<J, EnclosureError>,
        derivative: bool,
    ) -> Result<ScalarEnclosure, MetricError> {
        let evaluate = || {
            let range = equation(I::from_f64_bounds(lower, upper)?);
            let point = if lower == upper {
                range?
            } else {
                equation(I::point_f64(lower + (upper - lower) * 0.5)?)?
            };
            let range = match range {
                Ok(range) => range,
                Err(_) => {
                    // A wide interval may cross a denominator's zero even
                    // though its midpoint and smaller cells are admissible.
                    // Preserve uncertainty and let bounded isolation split.
                    let unknown = OutwardInterval {
                        lower: f64::NEG_INFINITY,
                        upper: f64::INFINITY,
                    };
                    return Ok(ScalarEnclosure {
                        value_estimate: midpoint(if derivative {
                            point.first
                        } else {
                            point.value
                        }),
                        derivative_estimate: midpoint(if derivative {
                            point.second
                        } else {
                            point.first
                        }),
                        value: unknown,
                        derivative: unknown,
                    });
                }
            };
            let (value, slope, point_value, point_slope) = if derivative {
                (range.first, range.second, point.first, point.second)
            } else {
                (range.value, range.first, point.value, point.first)
            };
            Ok(ScalarEnclosure {
                value_estimate: midpoint(point_value),
                derivative_estimate: midpoint(point_slope),
                value: OutwardInterval {
                    lower: value.lower_f64(),
                    upper: value.upper_f64(),
                },
                derivative: OutwardInterval {
                    lower: slope.lower_f64(),
                    upper: slope.upper_f64(),
                },
            })
        };
        evaluate().map_err(|_: EnclosureError| MetricError::AmbiguousRoot)
    }

    fn translation(&self, s: I, derivative: usize, anchor: [f64; 3]) -> Result<V, EnclosureError> {
        let mut result = [I::zero(); 3];
        if derivative > 3 {
            return Ok(result);
        }
        for (axis, result) in result.iter_mut().enumerate() {
            for power in (derivative..=3).rev() {
                let mut coefficient = I::point_f64(self.coefficient[axis][power])?;
                for order in 0..derivative {
                    coefficient = coefficient.scale_f64((power - order) as f64)?;
                }
                if derivative == 0 && power == 0 {
                    coefficient = coefficient.sub(I::point_f64(anchor[axis])?)?;
                }
                *result = result.mul(s)?.add(coefficient)?;
            }
        }
        Ok(result)
    }

    fn point(&self, s: I, derivative: usize) -> Result<V, EnclosureError> {
        add(
            self.translation(s, derivative, [0.0; 3])?,
            self.rotated(s, self.lever, derivative)?,
        )
    }

    fn rotated(&self, s: I, source: [f64; 3], derivative: usize) -> Result<V, EnclosureError> {
        // d^n(Ra Rb r) = sum binomial(n,k) Ra A^k Rb B^(n-k) r.
        // A and B need not commute. Derivatives through order three suffice
        // for value/first/second speed jets and their stationary equations.
        const BINOMIAL: [[f64; 4]; 4] = [
            [1.0, 0.0, 0.0, 0.0],
            [1.0, 1.0, 0.0, 0.0],
            [1.0, 2.0, 1.0, 0.0],
            [1.0, 3.0, 3.0, 1.0],
        ];
        let a = vector(self.rotation)?;
        let b = vector(self.correction)?;
        let mut result = [I::zero(); 3];
        for k in 0..=derivative {
            let mut term = vector(source)?;
            for _ in 0..(derivative - k) {
                term = I::cross(b, term)?;
            }
            term = rodrigues_rotate_enclosed(s, self.correction, term)?;
            for _ in 0..k {
                term = I::cross(a, term)?;
            }
            term = rodrigues_rotate_enclosed(s, self.rotation, term)?;
            result = add(result, scale(term, I::point_f64(BINOMIAL[derivative][k])?)?)?;
        }
        // UnitQuaternion normalizes its stored components when converting to
        // a rotation. Enclose that normalization too: accepted binary inputs
        // need not have an exactly unit real-arithmetic norm.
        let mut q4 = [I::zero(); 4];
        let mut norm_squared = I::zero();
        for (target, source) in q4.iter_mut().zip(self.quaternion) {
            *target = I::point_f64(source)?;
            norm_squared = norm_squared.add(target.square()?)?;
        }
        let qnorm = norm_squared.sqrt()?;
        for component in &mut q4 {
            *component = component.div(qnorm)?;
        }
        let [w, x, y, z] = q4;
        let q = [x, y, z];
        let cross = I::cross(q, result)?;
        add(
            result,
            scale(
                add(scale(cross, w)?, I::cross(q, cross)?)?,
                I::point_f64(2.0)?,
            )?,
        )
    }
}

fn midpoint(value: I) -> f64 {
    value.lower_f64() + (value.upper_f64() - value.lower_f64()) * 0.5
}
fn vector(value: [f64; 3]) -> Result<V, EnclosureError> {
    Ok([
        I::point_f64(value[0])?,
        I::point_f64(value[1])?,
        I::point_f64(value[2])?,
    ])
}
fn add(left: V, right: V) -> Result<V, EnclosureError> {
    Ok([
        left[0].add(right[0])?,
        left[1].add(right[1])?,
        left[2].add(right[2])?,
    ])
}
fn scale(value: V, scalar: I) -> Result<V, EnclosureError> {
    Ok([
        value[0].mul(scalar)?,
        value[1].mul(scalar)?,
        value[2].mul(scalar)?,
    ])
}
fn jets(value: V, first: V, second: V) -> [J; 3] {
    core::array::from_fn(|i| J {
        value: value[i],
        first: first[i],
        second: second[i],
    })
}
fn dot_jets(left: [J; 3], right: [J; 3]) -> Result<J, EnclosureError> {
    left[0]
        .mul(right[0])?
        .add(left[1].mul(right[1])?)?
        .add(left[2].mul(right[2])?)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn complete_expression_graph_encloses_independent_decimal90_oracles() {
        let rotations = [
            [0.4, -0.7, 1.1],
            [1.0e-6, -2.0e-6, 3.0e-6],
            [0.0; 3],
            [0.0, 0.0, 8.0],
        ];
        let corrections = [
            [-0.2, 0.3, 0.15],
            [-1.0e-6, 1.0e-6, 2.0e-6],
            [0.0; 3],
            [0.2, -0.1, 0.3],
        ];
        for line in include_str!("tests/fixtures/root_enclosure_decimal90.csv")
            .lines()
            .filter(|line| !line.starts_with('#'))
        {
            let values: std::vec::Vec<f64> = line
                .split(',')
                .map(|value| value.parse().unwrap())
                .collect();
            let case = values[0] as usize;
            let parameter = values[1];
            let expression = RootExpression {
                coefficient: [
                    [4_510_000.0, 20.0, -3.0, 1.0],
                    [451_000.0, -8.0, 7.0, -2.0],
                    [4_480_000.0, 5.0, -4.0, 2.0],
                ],
                rotation: rotations[case],
                correction: corrections[case],
                quaternion: [0.5; 4],
                duration: 1.25,
                lever: [1.25, -0.5, 0.125],
            };
            for width in [0.0, 1.0e-5, 0.125] {
                let lower = (parameter - width).max(0.0);
                let upper = (parameter + width).min(1.0);
                let gate = expression
                    .gate(
                        lower,
                        upper,
                        [4_510_000.0, 451_000.0, 4_480_000.0],
                        [0.6, -0.8, 0.0],
                    )
                    .unwrap();
                contains(gate.value, values[2]);
                contains(gate.derivative, values[3]);
                for (quantity, index) in [
                    (SpeedQuantity::Spatial3d, 5),
                    (SpeedQuantity::InstantaneousHorizontal, 8),
                    (SpeedQuantity::BodyLongitudinalSigned, 11),
                    (SpeedQuantity::BodyLongitudinalMagnitude, 14),
                ] {
                    let speed = expression
                        .speed(lower, upper, ReferenceEllipsoid::WGS84, quantity, Some(0.0))
                        .unwrap();
                    contains(speed.value, values[index]);
                    contains(speed.derivative, values[index + 1]);
                    let extremum = expression
                        .speed(lower, upper, ReferenceEllipsoid::WGS84, quantity, None)
                        .unwrap();
                    let index = if quantity == SpeedQuantity::BodyLongitudinalMagnitude {
                        11
                    } else {
                        index
                    };
                    contains(extremum.value, values[index + 1]);
                    contains(extremum.derivative, values[index + 2]);
                }
            }
        }
    }

    fn contains(interval: OutwardInterval, value: f64) {
        assert!(
            interval.lower <= value && value <= interval.upper,
            "[{:.17e}, {:.17e}] does not enclose {:.17e}",
            interval.lower,
            interval.upper,
            value
        );
    }
}
