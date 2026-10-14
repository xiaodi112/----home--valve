服务端下发任务示例 type 1任务下发 2阀门指令下发
/155/202607280001/leak/set
{
    "type":1,
    "taskNumber":"LEAK20260818161943202607280001999",
    "samplingFreq":120,
    "collectionDuration":900
}

taskNumber任务序号
samplingFreq采集频率 单位s
collectionDuration采集时长 单位s




收到任务示例
/155/202607280001/leak/reply
{
    "taskStatus":1, 
    "taskNumber":"LEAK20260818161943202607280001999"
}

数据上报示例
/155/202607280001/leak/reply
{
    "taskStatus":2,
    "taskNumber":"LEAK20260818161943202607280001999",
    "taskData":[
        {
            "collationTime":"2026-09-12 13:00:00",
            "temp":20.11,
            "pressure":13.33,
            "temp2":20.11,
            "pressure2":13.33
        },
        {
            "collationTime":"2026-09-12 13:00:10",
            "temp":20.11,
            "pressure":13.33,
            "temp2":20.11,
            "pressure2":13.33
        },
        {
            "collationTime":"2026-09-12 13:00:20",
            "temp":20.11,
            "pressure":13.33,
            "temp2":20.11,
            "pressure2":13.33
        },
        {
            "collationTime":"2026-09-12 13:00:30",
            "temp":20.11,
            "pressure1":13.33,
            "temp2":20.11,
            "pressure2":13.33
        },
        {
            "collationTime":"2026-09-12 13:00:40",
            "temp":20.11,
            "pressure":13.33,
            "temp2":20.11,
            "pressure2":13.33
        },
        {
            "collationTime":"2026-09-12 13:00:50",
            "temp":20.11,
            "pressure":13.33,
            "temp2":20.11,
            "pressure2":13.33
        }
    ]
}
collationTime 采集时间 时间格式为 yyyy-MM-dd HH:mm:ss
temp pressure 为物模型的标识符 有多个就传多个


任务完成示例
/155/202607280001/leak/reply
{
    "taskStatus":3,
    "taskNumber":"LEAK20260818161943202607280001999"
}

阀门指令下发示例
/155/202607280001/leak/set
{
    "type":2,
    "taskNumber":"LEAK20260818161943202607280001999",
    "valveCommand":1
}
 valveCommand 1开阀 2关阀

阀门指令下发回复示例
/155/202607280001/leak/reply
{
    "taskStatus":5,
    "taskNumber":"LEAK20260818161943202607280001999",
    "valveCmdStatus":1,
    "failReason":""
}
valveCmdStatus 1成功 2失败
失败时需要上传失败原因
failReason 0成功 1动作超时未到位 2阀门异常


单路传感器时：
服务端下发任务示例 type 1任务下发 2阀门指令下发
/155/202607280001/leak/set
{
    "type":1,
    "taskNumber":"LEAK20260818161943202607280001999",
    "samplingFreq":120,
    "collectionDuration":900
}

taskNumber任务序号
samplingFreq采集频率 单位s
collectionDuration采集时长 单位s




收到任务示例
/155/202607280001/leak/reply
{
    "taskStatus":1, 
    "taskNumber":"LEAK20260818161943202607280001999"
}

数据上报示例
/155/202607280001/leak/reply
{
    "taskStatus":2,
    "taskNumber":"LEAK20260818161943202607280001999",
    "taskData":[
        {
            "collationTime":"2026-09-12 13:00:00",
            "temp":20.11,
            "pressure":13.33,
        },
        {
            "collationTime":"2026-09-12 13:00:10",
            "temp":20.11,
            "pressure":13.33,
        },
        {
            "collationTime":"2026-09-12 13:00:20",
            "temp":20.11,
            "pressure":13.33,
        },
        {
            "collationTime":"2026-09-12 13:00:30",
            "temp":20.11,
            "pressure1":13.33,
        },
        {
            "collationTime":"2026-09-12 13:00:40",
            "temp":20.11,
            "pressure":13.33,
        },
        {
            "collationTime":"2026-09-12 13:00:50",
            "temp":20.11,
            "pressure":13.33,
        }
    ]
}
collationTime 采集时间 时间格式为 yyyy-MM-dd HH:mm:ss
temp pressure 为物模型的标识符 有多个就传多个


任务完成示例
/155/202607280001/leak/reply
{
    "taskStatus":3,
    "taskNumber":"LEAK20260818161943202607280001999"
}

阀门指令下发示例
/155/202607280001/leak/set
{
    "type":2,
    "taskNumber":"LEAK20260818161943202607280001999",
    "valveCommand":1
}
 valveCommand 1开阀 2关阀

阀门指令下发回复示例
/155/202607280001/leak/reply
{
    "taskStatus":5,
    "taskNumber":"LEAK20260818161943202607280001999",
    "valveCmdStatus":1,
    "failReason":""
}
valveCmdStatus 1成功 2失败
失败时需要上传失败原因
failReason 0成功 1动作超时未到位 2阀门异常