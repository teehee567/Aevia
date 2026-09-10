use core::fmt::Write;

pub fn append_status(text: &mut impl Write) -> core::fmt::Result {
    let snapshot = crate::tasks::power::snapshot();
    write!(text, " battery={}", snapshot.state.label())?;
    match snapshot.percent {
        Some(percent) => {
            write!(text, " battery-pct={percent}")?;
        }
        None => {
            text.write_str(" battery-pct=unknown")?;
        }
    }
    if let Some(mv) = snapshot.millivolts {
        write!(text, " battery-mv={mv}")?;
    }
    if let Some(soc) = snapshot.raw_soc {
        write!(text, " battery-soc-raw={soc:04x}")?;
    }
    if let Some(part) = snapshot.part {
        write!(text, " battery-part={part:02x}")?;
    }
    if let Some(status) = snapshot.charger_status {
        write!(
            text,
            " battery-regs={:02x}/{:02x}/{:02x}",
            status[0], status[1], status[2]
        )?;
    }
    write!(
        text,
        " battery-charger={} battery-gauge={} battery-errors={}",
        snapshot.charger_error.unwrap_or("ok"),
        snapshot.gauge_error.unwrap_or("ok"),
        snapshot.errors
    )?;
    Ok(())
}
