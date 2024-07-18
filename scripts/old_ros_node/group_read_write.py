from collections.abc import Iterable
from pathlib import Path

import dynio.dynamixel_controller as dxl
import numpy as np
from dynamixel_sdk import (
    GroupSyncWrite,
    GroupSyncRead,
    COMM_SUCCESS,
    DXL_HIWORD,
    DXL_LOBYTE,
    DXL_HIBYTE,
    DXL_LOWORD,
)
from numpy.typing import NDArray


def get_mx28_control_table_json_path() -> str:
    return str(Path(__file__).parent / "MX28-AR-2.json")


def group_sync_write(
    dxl_io: dxl.DynamixelIO,
    data_by_dynamixel_id: dict[int, int | NDArray],
    control_table_address: int = 132,
    data_length: int = 4,
):
    group_sync_write = GroupSyncWrite(
        dxl_io.port_handler,
        dxl_io.packet_handler[1],
        start_address=control_table_address,
        data_length=data_length,
    )

    for dxl_id, datum in data_by_dynamixel_id.items():
        if data_length == 1:
            datum = [DXL_LOBYTE(DXL_LOWORD(data_by_dynamixel_id[dxl_id]))]
        elif data_length == 2:
            datum = [
                DXL_LOBYTE(DXL_LOWORD(data_by_dynamixel_id[dxl_id])),
                DXL_HIBYTE(DXL_LOWORD(data_by_dynamixel_id[dxl_id])),
            ]
        else:
            datum = [
                DXL_LOBYTE(DXL_LOWORD(data_by_dynamixel_id[dxl_id])),
                DXL_HIBYTE(DXL_LOWORD(data_by_dynamixel_id[dxl_id])),
                DXL_LOBYTE(DXL_HIWORD(data_by_dynamixel_id[dxl_id])),
                DXL_HIBYTE(DXL_HIWORD(data_by_dynamixel_id[dxl_id])),
            ]
        group_sync_write.addParam(dxl_id, datum)

    comm_result = group_sync_write.txPacket()

    if comm_result != COMM_SUCCESS:
        print(f"{dxl_io.packet_handler.getTxRxResult(comm_result)}")


def group_sync_read(
    dxl_io: dxl.DynamixelIO,
    dxl_ids: Iterable[int],
    control_table_address: int = 132,
    data_length: int = 4,
) -> dict[int, int]:
    packet_handler = dxl_io.packet_handler[1]

    group_sync_read = GroupSyncRead(
        dxl_io.port_handler,
        packet_handler,
        start_address=control_table_address,
        data_length=data_length,
    )
    for dxl_id in dxl_ids:
        group_sync_read.addParam(dxl_id)

    comm_result = group_sync_read.txRxPacket()

    results = {}
    if comm_result != COMM_SUCCESS:
        print(f"{dxl_io.packet_handler.getTxRxResult(comm_result)}")
        return results

    for dxl_id in dxl_ids:
        if group_sync_read.isAvailable(dxl_id, control_table_address, data_length):
            results[dxl_id] = group_sync_read.getData(
                dxl_id, control_table_address, data_length
            )

    return results