//! The `asynch` module of each servo: the async twins of the typed register functions
//! and of the controller that `generate_servo!` makes. They follow the blocking ones
//! name for name, with two differences: the controller owns any
//! [`AsyncTransport`](crate::asynch::AsyncTransport), and its single-motor reads return
//! the value itself rather than a one-element `Vec`.

/// Generates `pub mod asynch` for a servo. Invoked by `generate_servo!`.
#[doc(hidden)]
#[macro_export]
macro_rules! generate_servo_async {
    ($servo_name:ident, $protocol:ident,
     $(($reg_name:ident, $reg_access:ident, $reg_addr:expr, $reg_type:ty, $conv:ident)),+ $(,)?
    ) => {
        paste::paste! {
            /// The async API of this servo, over any
            /// [`AsyncTransport`]($crate::asynch::AsyncTransport): one `async fn` per
            /// register, and a controller that owns the link.
            pub mod asynch {
                #[allow(unused_imports)]
                use super::*;

                $crate::generate_async_controller!($servo_name, $protocol);

                $(
                    $crate::generate_async_reg_access!($servo_name, $reg_name, $reg_access, $reg_addr, $reg_type, $conv);
                )+
            }
        }
    };
}

#[doc(hidden)]
#[macro_export]
macro_rules! generate_async_controller {
    ($servo_name:ident, $protocol:ident) => {
        paste::paste! {
            #[doc = concat!("Async controller of ", stringify!($servo_name), " motors: a protocol handler and the link to the bus, which it owns.")]
            ///
            /// Built with the servo's protocol; [`with_protocol_handler`](Self::with_protocol_handler)
            /// swaps in another handler (a post delay, say). Pass `&mut link` to keep the
            /// link for other uses.
            pub struct [<$servo_name:camel Controller>]<T> {
                dph: $crate::asynch::DynamixelProtocolHandler,
                transport: T,
            }

            impl<T: $crate::asynch::AsyncTransport> [<$servo_name:camel Controller>]<T> {
                /// A controller talking over `transport`, with this servo's protocol.
                pub fn new(transport: T) -> Self {
                    Self {
                        dph: $crate::asynch::DynamixelProtocolHandler::of_version(
                            $crate::protocol_version!($protocol),
                        )
                        .unwrap(),
                        transport,
                    }
                }

                /// Use `dph` for every exchange.
                pub fn with_protocol_handler(self, dph: $crate::asynch::DynamixelProtocolHandler) -> Self {
                    Self { dph, ..self }
                }

                /// The protocol handler.
                pub fn protocol_handler(&self) -> &$crate::asynch::DynamixelProtocolHandler {
                    &self.dph
                }

                /// The protocol handler, to change its settings.
                pub fn protocol_handler_mut(&mut self) -> &mut $crate::asynch::DynamixelProtocolHandler {
                    &mut self.dph
                }

                /// The link to the bus.
                pub fn transport_mut(&mut self) -> &mut T {
                    &mut self.transport
                }

                /// The link to the bus, back.
                pub fn into_transport(self) -> T {
                    self.transport
                }

                /// Ping motor `id`: whether it answered.
                pub async fn ping(&mut self, id: u8) -> $crate::Result<bool> {
                    self.dph.ping(&mut self.transport, id).await
                }

                /// Reboot motor `id`: whether it answered.
                pub async fn reboot(&mut self, id: u8) -> $crate::Result<bool> {
                    self.dph.reboot(&mut self.transport, id).await
                }

                /// Reset motor `id` to its factory settings.
                pub async fn factory_reset(
                    &mut self,
                    id: u8,
                    conserve_id_only: bool,
                    conserve_id_and_baudrate: bool,
                ) -> $crate::Result<()> {
                    self.dph
                        .factory_reset(&mut self.transport, id, conserve_id_only, conserve_id_and_baudrate)
                        .await
                }

                /// Read `length` raw bytes from address `addr` of motor `id`.
                pub async fn read_raw_data(&mut self, id: u8, addr: u8, length: u8) -> $crate::Result<Vec<u8>> {
                    self.dph.read(&mut self.transport, id, addr, length).await
                }

                /// Write raw bytes to address `addr` of motor `id`.
                pub async fn write_raw_data(&mut self, id: u8, addr: u8, data: &[u8]) -> $crate::Result<()> {
                    self.dph.write(&mut self.transport, id, addr, data).await
                }

                /// Same as [`read_raw_data`](Self::read_raw_data), plus the status packet's error field.
                pub async fn read_raw_data_with_error(
                    &mut self,
                    id: u8,
                    addr: u8,
                    length: u8,
                ) -> $crate::Result<(Vec<u8>, $crate::StatusError)> {
                    self.dph.read_with_error(&mut self.transport, id, addr, length).await
                }

                /// Same as [`write_raw_data`](Self::write_raw_data), plus the status packet's error field.
                pub async fn write_raw_data_with_error(
                    &mut self,
                    id: u8,
                    addr: u8,
                    data: &[u8],
                ) -> $crate::Result<$crate::StatusError> {
                    self.dph.write_with_error(&mut self.transport, id, addr, data).await
                }

                /// Read `length` raw bytes from address `addr` of every motor of `ids`.
                pub async fn sync_read_raw_data(
                    &mut self,
                    ids: &[u8],
                    addr: u8,
                    length: u8,
                ) -> $crate::Result<Vec<Vec<u8>>> {
                    self.dph.sync_read(&mut self.transport, ids, addr, length).await
                }

                /// Same as [`sync_read_raw_data`](Self::sync_read_raw_data), plus each motor's error field.
                pub async fn sync_read_raw_data_with_error(
                    &mut self,
                    ids: &[u8],
                    addr: u8,
                    length: u8,
                ) -> $crate::Result<Vec<(Vec<u8>, $crate::StatusError)>> {
                    self.dph.sync_read_with_error(&mut self.transport, ids, addr, length).await
                }

                /// Write `data[i]` to address `addr` of motor `ids[i]`, in one instruction.
                pub async fn sync_write_raw_data(
                    &mut self,
                    ids: &[u8],
                    addr: u8,
                    data: &[Vec<u8>],
                ) -> $crate::Result<()> {
                    self.dph.sync_write(&mut self.transport, ids, addr, data).await
                }
            }

            $crate::generate_async_protocol_options!($servo_name, $protocol);
        }
    };
}

#[doc(hidden)]
#[macro_export]
macro_rules! generate_async_protocol_options {
    ($servo_name:ident, v1) => {};
    ($servo_name:ident, v2) => {
        paste::paste! {
            impl<T: $crate::asynch::AsyncTransport> [<$servo_name:camel Controller>]<T> {
                /// Answer every `sync_read_*` with a fast sync read (instruction 0x8A).
                /// Needs firmware new enough to implement it (XL330: v46+).
                pub fn with_fast_sync_read(mut self) -> Self {
                    self.dph.set_fast_sync_read(true);
                    self
                }

                /// Turn the fast sync read routing of the `sync_read_*` methods on or off.
                pub fn set_fast_sync_read(&mut self, enabled: bool) {
                    self.dph.set_fast_sync_read(enabled);
                }
            }
        }
    };
}

#[doc(hidden)]
#[macro_export]
macro_rules! generate_async_reg_access {
    ($servo_name:ident, $reg_name:ident, r, $reg_addr:expr, $reg_type:ty, $conv:ident) => {
        $crate::generate_async_reg_read!($servo_name, $reg_name, $reg_addr, $reg_type, $conv);
    };
    ($servo_name:ident, $reg_name:ident, w, $reg_addr:expr, $reg_type:ty, $conv:ident) => {
        $crate::generate_async_reg_write!($servo_name, $reg_name, $reg_addr, $reg_type, $conv);
    };
    ($servo_name:ident, $reg_name:ident, rw, $reg_addr:expr, $reg_type:ty, $conv:ident) => {
        $crate::generate_async_reg_read!($servo_name, $reg_name, $reg_addr, $reg_type, $conv);
        $crate::generate_async_reg_write!($servo_name, $reg_name, $reg_addr, $reg_type, $conv);
    };
}

/// Decode the bytes of one register, refusing an answer of the wrong size rather than
/// panicking on it.
#[doc(hidden)]
#[macro_export]
macro_rules! async_decode {
    ($reg_type:ty, $bytes:expr) => {
        <$reg_type>::from_le_bytes(
            $bytes
                .try_into()
                .map_err(|_| $crate::CommunicationErrorKind::ParsingError)?,
        )
    };
}

#[doc(hidden)]
#[macro_export]
macro_rules! generate_async_reg_read {
    ($servo_name:ident, $reg_name:ident, $reg_addr:expr, $reg_type:ty, None) => {
        paste::paste! {
            #[doc = concat!("Read register *", stringify!($reg_name), "* (addr: ", stringify!($reg_addr), ", type: `", stringify!($reg_type), "`)")]
            pub async fn [<read_ $reg_name>](
                dph: &$crate::asynch::DynamixelProtocolHandler,
                port: &mut impl $crate::asynch::AsyncTransport,
                id: u8,
            ) -> $crate::Result<$reg_type> {
                let bytes = dph.read(port, id, $reg_addr, size_of::<$reg_type>() as u8).await?;
                Ok($crate::async_decode!($reg_type, bytes))
            }

            #[doc = concat!("Sync read register *", stringify!($reg_name), "* (addr: ", stringify!($reg_addr), ", type: `", stringify!($reg_type), "`)")]
            pub async fn [<sync_read_ $reg_name>](
                dph: &$crate::asynch::DynamixelProtocolHandler,
                port: &mut impl $crate::asynch::AsyncTransport,
                ids: &[u8],
            ) -> $crate::Result<Vec<$reg_type>> {
                let answers = dph.sync_read(port, ids, $reg_addr, size_of::<$reg_type>() as u8).await?;
                let mut values = Vec::with_capacity(answers.len());
                for bytes in answers {
                    values.push($crate::async_decode!($reg_type, bytes));
                }
                Ok(values)
            }

            impl<T: $crate::asynch::AsyncTransport> [<$servo_name:camel Controller>]<T> {
                #[doc = concat!("Read register *", stringify!($reg_name), "* (addr: ", stringify!($reg_addr), ", type: `", stringify!($reg_type), "`)")]
                pub async fn [<read_ $reg_name>](&mut self, id: u8) -> $crate::Result<$reg_type> {
                    [<read_ $reg_name>](&self.dph, &mut self.transport, id).await
                }

                #[doc = concat!("Sync read register *", stringify!($reg_name), "* (addr: ", stringify!($reg_addr), ", type: `", stringify!($reg_type), "`)")]
                pub async fn [<sync_read_ $reg_name>](&mut self, ids: &[u8]) -> $crate::Result<Vec<$reg_type>> {
                    [<sync_read_ $reg_name>](&self.dph, &mut self.transport, ids).await
                }
            }
        }
    };
    ($servo_name:ident, $reg_name:ident, $reg_addr:expr, $reg_type:ty, $conv:ident) => {
        paste::paste! {
            $crate::generate_async_reg_read!($servo_name, [<raw_ $reg_name>], $reg_addr, $reg_type, None);

            #[doc = concat!("Read register *", stringify!($reg_name), "* (addr: ", stringify!($reg_addr), ", converted by ", stringify!($conv), ")")]
            pub async fn [<read_ $reg_name>](
                dph: &$crate::asynch::DynamixelProtocolHandler,
                port: &mut impl $crate::asynch::AsyncTransport,
                id: u8,
            ) -> $crate::Result<<$conv as Conversion>::UsiType> {
                Ok($conv::from_raw([<read_raw_ $reg_name>](dph, port, id).await?))
            }

            #[doc = concat!("Sync read register *", stringify!($reg_name), "* (addr: ", stringify!($reg_addr), ", converted by ", stringify!($conv), ")")]
            pub async fn [<sync_read_ $reg_name>](
                dph: &$crate::asynch::DynamixelProtocolHandler,
                port: &mut impl $crate::asynch::AsyncTransport,
                ids: &[u8],
            ) -> $crate::Result<Vec<<$conv as Conversion>::UsiType>> {
                let raw = [<sync_read_raw_ $reg_name>](dph, port, ids).await?;
                Ok(raw.into_iter().map($conv::from_raw).collect())
            }

            impl<T: $crate::asynch::AsyncTransport> [<$servo_name:camel Controller>]<T> {
                #[doc = concat!("Read register *", stringify!($reg_name), "* (addr: ", stringify!($reg_addr), ", converted by ", stringify!($conv), ")")]
                pub async fn [<read_ $reg_name>](&mut self, id: u8) -> $crate::Result<<$conv as Conversion>::UsiType> {
                    [<read_ $reg_name>](&self.dph, &mut self.transport, id).await
                }

                #[doc = concat!("Sync read register *", stringify!($reg_name), "* (addr: ", stringify!($reg_addr), ", converted by ", stringify!($conv), ")")]
                pub async fn [<sync_read_ $reg_name>](&mut self, ids: &[u8]) -> $crate::Result<Vec<<$conv as Conversion>::UsiType>> {
                    [<sync_read_ $reg_name>](&self.dph, &mut self.transport, ids).await
                }
            }
        }
    };
}

#[doc(hidden)]
#[macro_export]
macro_rules! generate_async_reg_write {
    ($servo_name:ident, $reg_name:ident, $reg_addr:expr, $reg_type:ty, None) => {
        paste::paste! {
            #[doc = concat!("Write register *", stringify!($reg_name), "* (addr: ", stringify!($reg_addr), ", type: `", stringify!($reg_type), "`)")]
            pub async fn [<write_ $reg_name>](
                dph: &$crate::asynch::DynamixelProtocolHandler,
                port: &mut impl $crate::asynch::AsyncTransport,
                id: u8,
                value: $reg_type,
            ) -> $crate::Result<()> {
                dph.write(port, id, $reg_addr, &value.to_le_bytes()).await
            }

            #[doc = concat!("Sync write register *", stringify!($reg_name), "* (addr: ", stringify!($reg_addr), ", type: `", stringify!($reg_type), "`)")]
            pub async fn [<sync_write_ $reg_name>](
                dph: &$crate::asynch::DynamixelProtocolHandler,
                port: &mut impl $crate::asynch::AsyncTransport,
                ids: &[u8],
                values: &[$reg_type],
            ) -> $crate::Result<()> {
                let data: Vec<Vec<u8>> = values.iter().map(|v| v.to_le_bytes().to_vec()).collect();
                dph.sync_write(port, ids, $reg_addr, &data).await
            }

            impl<T: $crate::asynch::AsyncTransport> [<$servo_name:camel Controller>]<T> {
                #[doc = concat!("Write register *", stringify!($reg_name), "* (addr: ", stringify!($reg_addr), ", type: `", stringify!($reg_type), "`)")]
                pub async fn [<write_ $reg_name>](&mut self, id: u8, value: $reg_type) -> $crate::Result<()> {
                    [<write_ $reg_name>](&self.dph, &mut self.transport, id, value).await
                }

                #[doc = concat!("Sync write register *", stringify!($reg_name), "* (addr: ", stringify!($reg_addr), ", type: `", stringify!($reg_type), "`)")]
                pub async fn [<sync_write_ $reg_name>](&mut self, ids: &[u8], values: &[$reg_type]) -> $crate::Result<()> {
                    [<sync_write_ $reg_name>](&self.dph, &mut self.transport, ids, values).await
                }
            }
        }
    };
    ($servo_name:ident, $reg_name:ident, $reg_addr:expr, $reg_type:ty, $conv:ident) => {
        paste::paste! {
            $crate::generate_async_reg_write!($servo_name, [<raw_ $reg_name>], $reg_addr, $reg_type, None);

            #[doc = concat!("Write register *", stringify!($reg_name), "* (addr: ", stringify!($reg_addr), ", converted by ", stringify!($conv), ")")]
            pub async fn [<write_ $reg_name>](
                dph: &$crate::asynch::DynamixelProtocolHandler,
                port: &mut impl $crate::asynch::AsyncTransport,
                id: u8,
                value: <$conv as Conversion>::UsiType,
            ) -> $crate::Result<()> {
                [<write_raw_ $reg_name>](dph, port, id, $conv::to_raw(value)).await
            }

            #[doc = concat!("Sync write register *", stringify!($reg_name), "* (addr: ", stringify!($reg_addr), ", converted by ", stringify!($conv), ")")]
            pub async fn [<sync_write_ $reg_name>](
                dph: &$crate::asynch::DynamixelProtocolHandler,
                port: &mut impl $crate::asynch::AsyncTransport,
                ids: &[u8],
                values: &[<$conv as Conversion>::UsiType],
            ) -> $crate::Result<()> {
                let raw: Vec<$reg_type> = values.iter().map(|&v| $conv::to_raw(v)).collect();
                [<sync_write_raw_ $reg_name>](dph, port, ids, &raw).await
            }

            impl<T: $crate::asynch::AsyncTransport> [<$servo_name:camel Controller>]<T> {
                #[doc = concat!("Write register *", stringify!($reg_name), "* (addr: ", stringify!($reg_addr), ", converted by ", stringify!($conv), ")")]
                pub async fn [<write_ $reg_name>](&mut self, id: u8, value: <$conv as Conversion>::UsiType) -> $crate::Result<()> {
                    [<write_ $reg_name>](&self.dph, &mut self.transport, id, value).await
                }

                #[doc = concat!("Sync write register *", stringify!($reg_name), "* (addr: ", stringify!($reg_addr), ", converted by ", stringify!($conv), ")")]
                pub async fn [<sync_write_ $reg_name>](&mut self, ids: &[u8], values: &[<$conv as Conversion>::UsiType]) -> $crate::Result<()> {
                    [<sync_write_ $reg_name>](&self.dph, &mut self.transport, ids, values).await
                }
            }
        }
    };
}
