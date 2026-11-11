// Local EPL-2.0 correction: retain complete cubic segments across hierarchy boundaries.
use org_eclipse_elk_core::org::eclipse::elk::core::math::{kvector::KVector, kvector_chain::KVectorChain};

pub(super) fn append(
    output: &mut KVectorChain,
    last: Option<KVector>,
    source: KVector,
    controls: &KVectorChain,
    target: KVector,
) {
    // Each fragment's start belongs to the preceding fragment's end or to a bridge.
    if let Some(previous) = last {
        if (previous.x-source.x).abs()>1e-9 || (previous.y-source.y).abs()>1e-9 {
            output.add_vector(KVector::with_values(previous.x+(source.x-previous.x)/3.0,previous.y+(source.y-previous.y)/3.0));
            output.add_vector(KVector::with_values(previous.x+2.0*(source.x-previous.x)/3.0,previous.y+2.0*(source.y-previous.y)/3.0));
            output.add_vector(source);
        }
    }
    if controls.is_empty() {
        // A native straight fragment still contributes one complete cubic span.
        output.add_vector(source);
        output.add_vector(target);
    } else {
        output.add_all(&controls.to_array());
    }
    output.add_vector(target);
}
