//! Register access by name through a [`ServoDefinition`], over an [`AsyncTransport`]:
//! the async twin of the methods of [`ServoDefinition`] and of
//! [`servo::definition`](crate::servo::definition).
//!
//! They are free functions taking the definition first, since inherent async methods
//! would collide with the blocking ones. Values are integers, decoded from and laid out
//! in the definition's word order and each register's sign encoding, exactly as the
//! blocking ones do.

use alloc::collections::BTreeMap;

use super::{AsyncTransport, DynamixelProtocolHandler};
#[allow(unused_imports)]
use crate::prelude::*;
use crate::servo::definition::{answer_sync_read, ids, resolve, SyncWrite};
use crate::servo::{scan_timeout, RegisterError, RegisterInfo, ServoDefinition};
use crate::{Result, StatusError};

/// Read register `name` of motor `id` as an integer.
pub async fn read_register(
    definition: &ServoDefinition,
    dph: &DynamixelProtocolHandler,
    port: &mut impl AsyncTransport,
    id: u8,
    name: &str,
) -> Result<i64> {
    Ok(read_register_with_error(definition, dph, port, id, name)
        .await?
        .0)
}

/// Same as [`read_register`], plus the status packet's error field.
pub async fn read_register_with_error(
    definition: &ServoDefinition,
    dph: &DynamixelProtocolHandler,
    port: &mut impl AsyncTransport,
    id: u8,
    name: &str,
) -> Result<(i64, StatusError)> {
    definition.check_protocol(dph.protocol_version())?;
    let reg = definition.named(name)?;
    let (bytes, error) = dph.read_with_error(port, id, reg.addr, reg.size).await?;
    Ok((reg.decode(definition.info.word_order, &bytes)?, error))
}

/// Write `value` to register `name` of motor `id`. A value that does not fit the register
/// fails before anything reaches the bus.
pub async fn write_register(
    definition: &ServoDefinition,
    dph: &DynamixelProtocolHandler,
    port: &mut impl AsyncTransport,
    id: u8,
    name: &str,
    value: i64,
) -> Result<()> {
    write_register_with_error(definition, dph, port, id, name, value)
        .await
        .map(|_| ())
}

/// Same as [`write_register`], plus the status packet's error field.
pub async fn write_register_with_error(
    definition: &ServoDefinition,
    dph: &DynamixelProtocolHandler,
    port: &mut impl AsyncTransport,
    id: u8,
    name: &str,
    value: i64,
) -> Result<StatusError> {
    definition.check_protocol(dph.protocol_version())?;
    let reg = definition.named(name)?;
    let bytes = reg.encode(definition.info.word_order, value)?;
    dph.write_with_error(port, id, reg.addr, &bytes).await
}

/// Sync read register `name` from `motors`, each an id with its own definition, in one
/// instruction; see [`crate::servo::definition::sync_read_register`] for the rules. When
/// one of them has no Sync Read, the motors are read one at a time instead.
pub async fn sync_read_register(
    dph: &DynamixelProtocolHandler,
    port: &mut impl AsyncTransport,
    motors: &[(u8, ServoDefinition)],
    name: &str,
) -> Result<Vec<i64>> {
    Ok(sync_read_register_with_error(dph, port, motors, name)
        .await?
        .into_iter()
        .map(|(value, _)| value)
        .collect())
}

/// Same as [`sync_read_register`], plus each motor's error field.
pub async fn sync_read_register_with_error(
    dph: &DynamixelProtocolHandler,
    port: &mut impl AsyncTransport,
    motors: &[(u8, ServoDefinition)],
    name: &str,
) -> Result<Vec<(i64, StatusError)>> {
    if !answer_sync_read(motors) {
        let mut values = Vec::with_capacity(motors.len());
        for (id, definition) in motors {
            values.push(read_register_with_error(definition, dph, port, *id, name).await?);
        }
        return Ok(values);
    }
    let regs = resolve(dph.protocol_version(), motors, name)?;
    let Some(&(first, _)) = regs.first() else {
        return Ok(Vec::new());
    };
    let answers = dph
        .sync_read_with_error(port, &ids(motors), first.addr, first.size)
        .await?;
    regs.iter()
        .zip(answers)
        .map(|(&(reg, order), (bytes, error))| Ok((reg.decode(order, &bytes)?, error)))
        .collect()
}

/// Sync write `values` to register `name` of `motors`, one value per motor, in one
/// instruction. Every value is checked and encoded before anything is sent.
pub async fn sync_write_register(
    dph: &DynamixelProtocolHandler,
    port: &mut impl AsyncTransport,
    motors: &[(u8, ServoDefinition)],
    name: &str,
    values: &[i64],
) -> Result<()> {
    match SyncWrite::encode(dph.protocol_version(), motors, name, values)? {
        Some(write) => {
            dph.sync_write(port, &write.ids, write.addr, &write.data)
                .await
        }
        None => Ok(()),
    }
}

/// Which of `ids` answer, with their model number: one Model Number read per id, under a
/// timeout sized to `baudrate` (see [`scan_timeout`]); the link's own timeout is put back
/// afterwards.
pub async fn scan(
    definition: &ServoDefinition,
    dph: &DynamixelProtocolHandler,
    port: &mut impl AsyncTransport,
    ids: &[u8],
    baudrate: u32,
) -> Result<BTreeMap<u8, u16>> {
    definition.check_protocol(dph.protocol_version())?;
    let reg = definition.named("model_number")?;
    let timeout = port.timeout();
    port.set_timeout(scan_timeout(baudrate))?;
    let mut found = BTreeMap::new();
    for &id in ids {
        if let Some(model) = model_number(definition, dph, port, reg, id).await {
            found.insert(id, model);
        }
    }
    port.set_timeout(timeout)?;
    Ok(found)
}

/// Which of `ids` answer, with their model number: one broadcast ping at `baudrate`, then a
/// Model Number read of each id asked that answered. A servo that does not answer a
/// broadcast ping is refused before anything is sent.
pub async fn broadcast_scan(
    definition: &ServoDefinition,
    dph: &DynamixelProtocolHandler,
    port: &mut impl AsyncTransport,
    ids: &[u8],
    baudrate: u32,
) -> Result<BTreeMap<u8, u16>> {
    definition.check_protocol(dph.protocol_version())?;
    let reg = definition.named("model_number")?;
    if !definition.info.supports_broadcast_ping {
        return Err(RegisterError::BroadcastPing(definition.name).into());
    }
    let answered = dph.broadcast_ping(port, baudrate).await?;
    let mut found = BTreeMap::new();
    for &id in ids.iter().filter(|id| answered.contains(id)) {
        if let Some(model) = model_number(definition, dph, port, reg, id).await {
            found.insert(id, model);
        }
    }
    Ok(found)
}

async fn model_number(
    definition: &ServoDefinition,
    dph: &DynamixelProtocolHandler,
    port: &mut impl AsyncTransport,
    reg: RegisterInfo,
    id: u8,
) -> Option<u16> {
    let bytes = dph.read(port, id, reg.addr, reg.size).await.ok()?;
    let model = reg.decode(definition.info.word_order, &bytes).ok()?;
    Some(model as u16)
}

/// Give motor `id` the id `new_id`, after turning its torque off and opening its lock (if
/// it has one), as [`ServoDefinition::change_id`] does. A refusal fails with a
/// [`StatusPacketError`](crate::StatusPacketError).
pub async fn change_id(
    definition: &ServoDefinition,
    dph: &DynamixelProtocolHandler,
    port: &mut impl AsyncTransport,
    id: u8,
    new_id: u8,
) -> Result<()> {
    open_eeprom(definition, dph, port, id).await?;
    write_checked(definition, dph, port, id, "id", new_id.into()).await
}

/// Set motor `id` to talk at `baudrate`, one of the definition's `baudrates`, as
/// [`ServoDefinition::change_baudrate`] does. The link stays at the old rate.
pub async fn change_baudrate(
    definition: &ServoDefinition,
    dph: &DynamixelProtocolHandler,
    port: &mut impl AsyncTransport,
    id: u8,
    baudrate: u32,
) -> Result<()> {
    let &(_, value) = definition
        .info
        .baudrates
        .iter()
        .find(|&&(rate, _)| rate == baudrate)
        .ok_or(RegisterError::Baudrate {
            servo: definition.name,
            baudrate,
        })?;
    open_eeprom(definition, dph, port, id).await?;
    write_checked(definition, dph, port, id, "baud_rate", value.into()).await
}

async fn open_eeprom(
    definition: &ServoDefinition,
    dph: &DynamixelProtocolHandler,
    port: &mut impl AsyncTransport,
    id: u8,
) -> Result<()> {
    write_checked(definition, dph, port, id, "torque_enable", 0).await?;
    if definition.register("lock").is_some() {
        write_checked(definition, dph, port, id, "lock", 0).await?;
    }
    Ok(())
}

async fn write_checked(
    definition: &ServoDefinition,
    dph: &DynamixelProtocolHandler,
    port: &mut impl AsyncTransport,
    id: u8,
    name: &str,
    value: i64,
) -> Result<()> {
    let status = write_register_with_error(definition, dph, port, id, name, value).await?;
    definition.check_status(id, status)
}
