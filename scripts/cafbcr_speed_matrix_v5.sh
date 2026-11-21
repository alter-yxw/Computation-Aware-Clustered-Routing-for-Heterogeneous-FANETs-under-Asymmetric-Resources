#!/usr/bin/env bash
# v5: explicit ARP identity, eight public objects, standalone author-started batch.
# Clean CSV46; native70 and commands in audit.log; rolling host-wide maximum20.
set -euo pipefail

# ========== 配置区域 ==========
# Three CA HELLO modes plus five public baselines; MOGNN uses legacy at every scale; AODV is excluded.
# FBCR (the authors' prior work) is not part of this public release.
ALGORITHMS=(cafbcr ca-qbcr ca-bcr olsr qrcf cfr-eng mognn)
SIZES=(200)
SHADOW_SIGMAS=(4)
SEEDS=(20261106)
RNG_RUNS=(1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20)  # 固定Seed、20个独立RngRun；本组点数见--plan。
SPEED_RANGES=(10:20 20:30 30:40 40:50 50:60 20:60)
ASYM_LEVEL=4
ASYM_RANGE_PROFILE=linked
SIM_TIME=350
TASK_START=30
TASK_STOP=150
UDP_START=30
UDP_STOP=150
UDP_FLOWS=50
PATH_LOSS_EXPONENT=2
# Shared ARP defaults are explicit for all nine objects.
ARP_DEAD_REPROBE=0
ARP_MAX_RETRIES=3
ARP_WAIT_REPLY_MS=1000
CA_LOOP_POLICY=alternate-revisit
CA_PROXY_RENEWAL_POLICY=direct-feedback
CA_PROXY_RELAY_POLICY=ack-stagger
CA_CONTROL_UNICAST_POLICY=one-hop
CA_DIRECT_CM_POLICY=legacy
# MOGNN remains learned/fixed/legacy at every scale; batch-v1 is not used.
MO_MODEL=contrib/mognn/models/BAT800.txt
MO_SLOT_SECONDS=0.01
MO_LINK_BPS=6000000
MO_HELLO_INTERVAL=1
MO_HOLD_TIME=4
MO_RELAY_MODE=legacy
MO_RELAY_WINDOW_MS=20
MO_RELAY_SUPPRESS_DUPLICATES=false
MO_SERVER_HELLO_SCALE=1
BUILD_FIRST=0
BUILD_JOBS=2
MAX_JOBS=20
# 与Speed/asym及计算权重工具共享主机锁，不叠加运行池。
GLOBAL_LOCK_FILE="/tmp/cafbcr-matrix-${UID}.lock"
CSV_FILE="Result/cafbcr_speed_v5.csv"
# 可选：--import-reference=原生Speed.csv仅合并当前计划匹配的同Build行，仿真0。
# 首批可复用N200的24行，实际新增96；扩Run后可再次导入更多N200参考行。

# 下方为独立运行逻辑；调度设置不改变场景身份。

die() { echo "错误：$*" >&2; exit 2; }
record_event() {
    local context=${CONTEXT[$2]:-$'\t-\t-\t-\t-\t-\t-\t-\t-'}
    printf '%(%Y-%m-%dT%H:%M:%S%z)T\t%s%s\t%s\t%s\t%s\n' -1 "$1" "$context" "$2" "$BUILD_ID" "$3" >> "$STATUS_FILE"
}
build_id() { sha256sum build/scratch/ns3.40-ca-fbcr-test-default build/lib/*.so | sha256sum | cut -d ' ' -f 1; }
make_args() {
    local algo=$1 n=$2 minimum=$3 maximum=$4 level=$5 ranges=$6 proxy=$7 gate=$8 seed=$9 run=${10} sigma=${11}
    local protocol=$algo hello=fuzzy relay_policy=legacy
    ((proxy)) && relay_policy=$CA_PROXY_RELAY_POLICY
    case $algo in
        ca-qbcr) protocol=cafbcr; hello=qlearning ;;
        ca-bcr) protocol=cafbcr; hello=fixed ;;
    esac
    LABEL=$algo
    ARGS=("--protocol=$protocol" "--size=$n" "--seed=$seed" "--RngRun=$run"
        "--MinSpeed=$minimum" "--MaxSpeed=$maximum" "--time=$SIM_TIME"
        "--taskGenStart=$TASK_START" "--taskGenStop=$TASK_STOP" --taskStartMode=bounded-1s
        "--udpStart=$UDP_START" "--udpStop=$UDP_STOP" "--numTestFlows=$UDP_FLOWS"
        --fanetMode=heterogeneous-compute --caVariant=hardfix "--helloMethod=$hello"
        --taskSelection=fixed --enableComputeTask=true --syncQueueLoad=true --taskWire=header-v1
        --udpDemand=fixed-attempts --requireTaskDeadlineCoverage=1
        "--asymLevel=$level" "--asymRangeProfile=$ranges" --statsInterval=5 --phyHeaderBoundaryGuard=1
        --channelModel=lognormal "--pathLossExponent=$PATH_LOSS_EXPONENT"
        "--shadowSigmaDb=$sigma" --shadowCorrDistance=20 --shadowUpdate=0.1
        --channelMaxRange=0 --shadowStreamBase=8000000 --shadowPairingProcess=1
        --channelPairingAudit=0 --shadowTracePairs=0 --csvOnly=1 --csvConfiguration=1
        --csvScenario=1 --csvUdpMetrics=1 --csvRunDiagnostics=0 --csvFailureSummary=0
        "--arpDeadReprobe=$ARP_DEAD_REPROBE" "--arpMaxRetries=$ARP_MAX_RETRIES"
        "--arpWaitReplyMs=$ARP_WAIT_REPLY_MS"
        "--csvBuildId=$BUILD_ID" --controlMeter=l3 --verbose=false --traceDrops=false --cleanShutdown=false)
    case $algo in
        cafbcr|ca-qbcr|ca-bcr) [[ $algo != cafbcr ]] || LABEL=ca-fbcr
            ((proxy)) || LABEL+=-noproxy
            ((gate)) || LABEL+=-norange
            ARGS+=(--caInitialSelector=abc "--caProxyJoin=$proxy" "--caRangeGating=$gate"
                "--caProxyRenewal=$proxy" "--caProxyRenewalPolicy=$CA_PROXY_RENEWAL_POLICY"
                "--caProxyRelayPolicy=$relay_policy" "--caControlUnicastPolicy=$CA_CONTROL_UNICAST_POLICY"
                "--caDirectCmPolicy=$CA_DIRECT_CM_POLICY"
                "--caLoopPolicy=$CA_LOOP_POLICY" --caLinkProbes=0 --caLinkGate=distance) ;;
        olsr) ARGS+=(--olsrHelloInterval=0.25) ;;
        qrcf) ARGS+=(--qrcfPreset=outcome-bellman) ;;
        cfr-eng) LABEL=cfr-eng-destination; ARGS+=(--cfrEngCompute=destination) ;;
        mognn) LABEL=mognn-fixed-destination
            local model=$MO_MODEL
            [[ $model == /* ]] || model="$PWD/$model"
            ARGS+=("--moModel=$model" "--moSlotSeconds=$MO_SLOT_SECONDS" "--moLinkBps=$MO_LINK_BPS"
                "--moHelloInterval=$MO_HELLO_INTERVAL" "--moHoldTime=$MO_HOLD_TIME"
                "--moRelayMode=$MO_RELAY_MODE" "--moRelayWindowMs=$MO_RELAY_WINDOW_MS"
                "--moRelaySuppressDuplicates=$MO_RELAY_SUPPRESS_DUPLICATES"
                "--moServerHelloScale=$MO_SERVER_HELLO_SCALE") ;;
        *) die "未知协议：$algo（AODV不纳入）" ;;
    esac
    printf -v RUN_CONFIG '%q ' "${ARGS[@]}"
}
add_group() {
    local algo=$1 n=$2 minimum=$3 maximum=$4 level=$5 ranges=$6 proxy=$7 gate=$8 sigma=$9 seed run group preview identifier key
    for seed in "${SEEDS[@]}"; do
    for run in "${RNG_RUNS[@]}"; do
        make_args "$algo" "$n" "$minimum" "$maximum" "$level" "$ranges" "$proxy" "$gate" "$seed" "$run" "$sigma"
        group="$LABEL:$n:$minimum:$maximum:$level:$ranges:$sigma"
        if [[ $MODE == plan ]]; then
            key="$group:$seed:$run"
            [[ ! ${PLANNED[$key]+yes} ]] || die "重复组合：$key"
            PLANNED[$key]=1
            printf './ns3 run --no-build %q\n' "scratch/ca-fbcr-test $RUN_CONFIG --resultsCsv=$CSV_FILE"
        else
            if [[ ! ${PREVIEWS[$group]+yes} ]]; then
                if ! preview=$(./ns3 run --no-build "scratch/ca-fbcr-test $RUN_CONFIG --dryRun=1 --resultsCsv=$WORK/unused.csv" 2>&1); then
                    printf '%s\n' "$preview" >&2
                    die "零节点参数预检失败：$group"
                fi
                [[ $preview == *'nodesCreated=0'* ]] || die "预检未证明零节点：$group"
                preview=$(sed -n 's/^\[Scenario\] //p' <<< "$preview")
                identifier=$(sed -n 's/^{"schema":"cafbcr-scenario-v1","id":"\([^"]*\)",.*/\1/p' <<< "$preview")
                [[ $identifier == cafbcr-scenario-v1-* ]] || die "缺少新场景合同，请重新编译：$group"
                PREVIEWS[$group]=$preview
                IDS[$group]=$identifier
                if [[ $MODE == check ]]; then echo "预检通过 $LABEL N=$n 速度=$minimum~$maximum m/s 非对称=L$level 阴影σ=$sigma dB 范围=$ranges 场景=$identifier"; fi
            fi
            identifier=${IDS[$group]}
            key="$identifier:$seed:$run"
            [[ ! ${PLANNED[$key]+yes} ]] || die "重复组合：$group seed=$seed run=$run"
            PLANNED[$key]=1
            printf '%s\t%s\t%s\t%s\n' "$key" "$seed" "$run" "${PREVIEWS[$group]}" >> "$PLAN"
            ORDER+=("$key")
            COMMANDS[$key]=$RUN_CONFIG
            LABELS[$key]="$LABEL N=$n 速度=$minimum~$maximum m/s 非对称=L$level 阴影σ=$sigma dB 范围=$ranges Seed=$seed RngRun=$run"
            printf -v 'CONTEXT[$key]' '\t%s\t%s\t%s\tL%s\t%s\t%s\t%s\t%s' "$LABEL" "$n" "$minimum~$maximum" "$level" "$sigma" "$ranges" "$seed" "$run"
        fi
        TOTAL=$((TOTAL+1))
    done; done
}
collect_one() {
    local pid rc key
    while :; do
        for pid in "${!JOB_DIR[@]}"; do
            kill -0 "$pid" 2>/dev/null && continue
            if wait "$pid"; then rc=0; else rc=$?; fi
            kill -KILL -- "-$pid" 2>/dev/null || true
            key=${JOB_KEY[$pid]}
            printf '%s\n' "$rc" > "${JOB_DIR[$pid]}/exit-code.txt"
            record_event EXIT "$key" "$rc"
            if ((rc)); then
                tail -n 20 -- "${JOB_DIR[$pid]}/stdout.log" "${JOB_DIR[$pid]}/stderr.log" >&2
                die "${LABELS[$key]}失败（退出码=$rc）；保留失败资料，下次重试同一组合"
            fi
            [[ $(build_id) == "$BUILD_ID" ]] || die '运行期间构建改变；保留总CSV并另设新文件'
            if ! csv_tool append "${JOB_DIR[$pid]}/result.csv" "$PLAN" "$BUILD_ID" "$CSV_FILE" "$key" "${COMMANDS[$key]}"; then
                printf 'VALIDATION_FAILED\n' > "${JOB_DIR[$pid]}/status.txt"
                record_event VALIDATION_FAILED "$key" 0
                die "${LABELS[$key]}结果验收失败"
            fi
            record_event SAVED "$key" 0
            completed=$((completed+1))
            echo "完成 [$((completed+skipped))/$TOTAL] ${LABELS[$key]}"
            rm -rf -- "${JOB_DIR[$pid]}"
            unset 'JOB_DIR[$pid]' 'JOB_KEY[$pid]'
            return
        done
        sleep 0.2
    done
}
cleanup() {
    local rc=$? pid child_rc failed_dir
    trap - EXIT INT TERM HUP
    for pid in "${!JOB_DIR[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done
    if ((${#JOB_DIR[@]})); then sleep 1; fi
    for pid in "${!JOB_DIR[@]}"; do
        kill -KILL -- "-$pid" 2>/dev/null || true
        if [[ ! -f ${JOB_DIR[$pid]}/exit-code.txt ]]; then
            if wait "$pid" 2>/dev/null; then child_rc=0; else child_rc=$?; fi
            printf '%s\n' "$child_rc" > "${JOB_DIR[$pid]}/exit-code.txt"
            printf 'cancelled-by-parent\n' > "${JOB_DIR[$pid]}/status.txt"
        else
            wait "$pid" 2>/dev/null || true
        fi
    done
    if ((rc)) && [[ -n ${FAILURE_ROOT:-} && -d $WORK ]]; then
        mkdir -p -- "$FAILURE_ROOT"
        failed_dir=$(mktemp -d "$FAILURE_ROOT/session-XXXXXXXX")
        mv -- "$WORK" "$failed_dir/work"
        printf '%s\n' "$rc" > "$failed_dir/launcher-exit-code.txt"
        record_event ABORT - "$rc"
        echo "失败／中断资料保留：$failed_dir" >&2
    else
        rm -rf -- "$WORK"
    fi
    exit "$rc"
}

csv_tool() {
    python3 -B - "$@" <<'PY'
import csv,hashlib,io,json,math,os,sys,tempfile
from pathlib import Path
operation,filename,planfile,build=sys.argv[1:5]
schema='cafbcr-scenario-v1'
base='Algorithm,Nodes,Speed(m/s),Seed,PDR(%),AvgDelay(ms),AvgHops,ValidFlows,Throughput(kbps),OverHeadRatio,ControlThroughput,HelloRatio,AvgLinkLen(s),TotalEnergy(J),AvgEnergy(J),DeadNodes,IsClusteringAlgo,TotalClusterNum,AvgClusterNum,AvgClusteringRatio(%),AvgClusterSize,AvgClusterLifetime(s),AvgClusterSwitchCount(/node),AvgRoleChangeCount(/node),TasksGenerated,TasksCompleted,TaskCompletionRate(%),AvgResponseTime(ms),AvgProcessingDelay(ms),AvgQueueWaitTime(ms),DeadlineMissRate(%),DeadlineSatisfactionRate(%),TasksOnTime,TasksLate,TasksUnconfirmedDue,TasksPending,TasksReceived,TasksProcessed,UDPAttempts,UDPReceived,UDPAttemptPDR(%),SimTime(s),RngRun,ChannelModel,PathLossExponent,ShadowSigma(dB),TaskSelection,BuildId,DeadlineWindowComplete,TaskGenStart(s),TaskGenStop(s),TaskStartMode,UDPWindowMode,UDPStart(s),UDPStop(s),UDPFlows,UDPInterval(s),UDPPacketBytes,ControlMeter'.split(',')
extra='ScenarioSchema,ScenarioId,MinSpeed(m/s),MaxSpeed(m/s),AsymLevel,AsymRangeProfile,ScenarioConfig'.split(',')
udp_metrics='UDPMetricProfile,UDPDelaySamples,UDPDelaySum(ns),UDPMeanDelay(ms)'.split(',')
udp_profile='cafbcr-udp-unique-delay-v1'
expected_header=base+udp_metrics+extra
def identifier(config):
    h=14695981039346656037
    for b in json.dumps(config,sort_keys=True,separators=(',',':'),ensure_ascii=False).encode():
        h=((h^b)*1099511628211)&((1<<64)-1)
    return schema+'-'+format(h,'016x')
def close(a,b):
    try:
        x,y=float(a),float(b)
        return math.isfinite(x) and math.isfinite(y) and math.isclose(x,y,rel_tol=1e-10,abs_tol=1e-9)
    except (ValueError,TypeError): return a==b
def need(ok,message):
    if not ok: raise ValueError(message)
plan={}
for line in Path(planfile).read_text().splitlines():
    key,seed,run,preview=line.split('\t',3)
    p=json.loads(preview); config=p['config']
    need(p['schema']==schema and p['id']==identifier(config),'invalid preflight identity')
    need(all(k in config for k in ('arpDeadReprobe','arpMaxRetries','arpWaitReplyMs')),'ARP effective fields missing; rebuild the frozen CA entry')
    need(config.get('UDPMetricProfile')==udp_profile,'UDP measurement profile missing/changed; rebuild CA entry')
    need(key==p['id']+':'+seed+':'+run and key not in plan,'duplicate/invalid planned case')
    plan[key]=config
def read(path):
    path=Path(path)
    if not path.exists() or path.stat().st_size==0: return [],None
    need(path.read_bytes().endswith(b'\n'),'CSV truncated: missing final newline')
    with path.open(newline='') as f:
        reader=csv.DictReader(f,strict=True); header=reader.fieldnames
        need(len(header)==len(expected_header) and set(header)==set(expected_header),'CSV schema differs')
        rows=list(reader)
    keys=set()
    for row in rows:
        need(set(row)==set(expected_header) and all(v not in (None,'') for v in row.values()),'missing/extra fields')
        key=row['ScenarioId']+':'+row['Seed']+':'+row['RngRun']
        need(key in plan and key not in keys,'unknown or duplicate case: '+key); keys.add(key)
        cfg=plan[key]
        need(row['BuildId']==build and row['ScenarioSchema']==schema,'build/schema mismatch')
        need(json.loads(row['ScenarioConfig'])==cfg,'effective scenario changed')
        need(identifier(cfg)==row['ScenarioId'],'scenario identity mismatch')
        for k,v in cfg.items():
            if k in row: need(close(row[k],v),'configuration mismatch: '+k)
        lo,hi=row['Speed(m/s)'].split('~')
        need(close(lo,cfg['MinSpeed(m/s)']) and close(hi,cfg['MaxSpeed(m/s)']),'speed label mismatch')
        text_fields={'Algorithm','Speed(m/s)','IsClusteringAlgo','ChannelModel','TaskSelection','BuildId','TaskStartMode','UDPWindowMode','ControlMeter',*extra}
        for k in base:
            if k in text_fields or row[k]=='N/A': continue
            value=float(row[k]); need(math.isfinite(value) and value>=0,'invalid numeric value: '+k)
        def integer(k):
            v=float(row[k]); need(math.isfinite(v) and v>=0 and v.is_integer(),'invalid count: '+k); return int(v)
        g,on,late,due,pending,done,received,processed=[integer(k) for k in ('TasksGenerated','TasksOnTime','TasksLate','TasksUnconfirmedDue','TasksPending','TasksCompleted','TasksReceived','TasksProcessed')]
        need(g==on+late+due+pending and done==on+late,'task terminal counts disagree')
        need(0<=done<=processed<=received<=g,'task service/confirmation counts disagree')
        need(integer('DeadlineWindowComplete')==1 and pending==0,'incomplete deadline window')
        attempts,rx=integer('UDPAttempts'),integer('UDPReceived')
        need(rx<=attempts,'UDP received exceeds plan')
        need(row['UDPMetricProfile']==udp_profile,'UDP measurement profile mismatch')
        def exact_uint(k):
            value=row[k]
            need(value.isascii() and value.isdecimal(),'invalid exact integer: '+k)
            value=int(value); need(value<=2**64-1,'UDP integer overflow: '+k)
            return value
        samples=exact_uint('UDPDelaySamples'); delay_ns=exact_uint('UDPDelaySum(ns)')
        need(samples==rx,'UDP delay samples differ from unique deliveries')
        # The existing ledger observes Tx and Rx in [30s, simulation end),
        # including packets received after the configured UDP generation stop.
        max_delay_ns=round((float(cfg['SimTime(s)'])-30)*1e9)
        need(delay_ns<=samples*max(0,max_delay_ns),'UDP delay sum exceeds observation window')
        need(row['UDPMeanDelay(ms)']=='N/A' if not samples else
             close(row['UDPMeanDelay(ms)'],delay_ns/samples/1e6),'UDP mean delay mismatch')
        start=round(float(cfg['UDPStart(s)'])*1e9); stop=round(float(cfg['UDPStop(s)'])*1e9)
        flows=int(cfg['UDPFlows']); interval=500000000
        planned=sum(max(0,(stop-start-(interval*i//flows)+interval-1)//interval) for i in range(flows))
        need(attempts==planned,'UDP planned denominator changed')
        index=expected_tasks=0; nodes=int(cfg['Nodes'])
        t0=round(float(cfg['TaskGenStart(s)'])*1e9); t1=round(float(cfg['TaskGenStop(s)'])*1e9)
        for kind in ('COMMAND','RELAY','TASK','SWARM'):
            step=round(float(cfg[kind+'.TaskInterval(s)'])*1e9)
            for _ in range(int(cfg[kind+'.Count'])):
                first=t0+index*1000000000//nodes
                if cfg['ComputeEnabled']=='1' and cfg[kind+'.GenerateTasks']=='1':
                    expected_tasks+=max(0,(t1-first+step-1)//step)
                index+=1
        need(g==expected_tasks,'task generated denominator changed')
        for count,denom,field in ((rx,attempts,'UDPAttemptPDR(%)'),(done,g,'TaskCompletionRate(%)'),(on,g,'DeadlineSatisfactionRate(%)'),(late+due,g,'DeadlineMissRate(%)')):
            need(row[field]=='N/A' if not denom else close(row[field],100*count/denom),'ratio mismatch: '+field)
        for k in base:
            if k.endswith('(%)') and row[k]!='N/A': need(0<=float(row[k])<=100,'percentage out of range: '+k)
    return rows,header

# The numerical strings stay native. Only the human-facing L label and two
# historical FlowMonitor column names, plus the control unit label, change.
front='Algorithm,Nodes,Speed(m/s),AsymLevel,ShadowSigma(dB),AsymRangeProfile,RngRun,Seed'.split(',')
metrics=[x for x in base[:base.index('SimTime(s)')] if x not in front]
metrics.insert(0,'UDPMeanDelay(ms)')
metrics.remove('UDPAttemptPDR(%)'); metrics.insert(0,'UDPAttemptPDR(%)')
aliases={'PDR(%)':'FlowPDR(%)','AvgDelay(ms)':'FlowMeanDelay(ms)','ControlThroughput':'ControlThroughput(kB/s)'}
columns=front+[aliases.get(x,x) for x in metrics]
def project(row):
    out={aliases.get(k,k):v for k,v in row.items() if k in front or k in metrics}
    out['AsymLevel']='L'+row['AsymLevel']
    return out
def render(rows):
    if not rows: return ''
    stream=io.StringIO()
    writer=csv.DictWriter(stream,fieldnames=columns,lineterminator='\n')
    writer.writeheader(); writer.writerows(project(row) for row in rows)
    return stream.getvalue()
def atomic_text(path,text):
    path=Path(path); path.parent.mkdir(parents=True,exist_ok=True)
    with tempfile.NamedTemporaryFile('w',dir=path.parent,prefix='.'+path.name+'.',delete=False) as stream:
        temporary=Path(stream.name)
        stream.write(text); stream.flush(); os.fsync(stream.fileno())
    try: os.replace(temporary,path)
    finally:
        if temporary.exists(): temporary.unlink()
    descriptor=os.open(path.parent,os.O_RDONLY|os.O_DIRECTORY)
    try: os.fsync(descriptor)
    finally: os.close(descriptor)
def native_key(row): return row['ScenarioId']+':'+row['Seed']+':'+row['RngRun']
def row_hash(row):
    return hashlib.sha256(json.dumps(row,sort_keys=True,separators=(',',':'),ensure_ascii=False).encode()).hexdigest()
def audit_record(row,command):
    return json.dumps({'schema':'cafbcr-result-audit-v1','key':native_key(row),'row_sha256':row_hash(row),
                       'command':command,'row':row},sort_keys=True,separators=(',',':'),ensure_ascii=False)+'\n'
def bundle(path,repair=False):
    path=Path(path); audit=Path(str(path)+'.audit.log')
    present=path.read_text() if path.exists() else ''
    if not audit.exists():
        need(not present,'clean CSV needs its audit log; native 70-column CSV requires explicit import')
        return [],''
    raw=audit.read_text()
    need(not raw or raw.endswith('\n'),'incomplete audit log; preserve files for recovery')
    records=[]
    for line in raw.splitlines():
        entry=json.loads(line)
        need(entry['schema']=='cafbcr-result-audit-v1','audit schema changed')
        row=entry['row']
        need(entry['key']==native_key(row) and entry['row_sha256']==row_hash(row),'audit row identity/digest differs')
        records.append(row)
    # Reuse the complete native guard, not a weaker clean-CSV resume key.
    with tempfile.TemporaryDirectory(prefix='cafbcr-audit-check-') as folder:
        native=Path(folder)/'rows.csv'
        with native.open('w',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=expected_header,lineterminator='\n')
            writer.writeheader(); writer.writerows(records)
        read(native)
    complete=render(records)
    if present != complete:
        # The audit log commits first. An old complete CSV prefix is the only
        # automatically recoverable mismatch; changed values never get hidden.
        count=max(0,len(list(csv.reader(io.StringIO(present))))-1) if present else 0
        need(count<len(records) and present==render(records[:count]),'clean CSV differs from audited native values')
        need(repair,'CSV projection lags its audit log; use --resume --no-build')
        atomic_text(path,complete)
        print('已从完整审计日志恢复CSV显示行；没有重跑仿真',file=sys.stderr)
    return records,raw

try:
    if operation in ('validate','resume-keys'):
        rows,_=bundle(filename,repair=operation=='resume-keys')
        for row in rows: print(native_key(row))
    elif operation=='append':
        destination,required,command=sys.argv[5:8]
        rows,_=read(filename); need(len(rows)==1,'child must write exactly one row')
        row=rows[0]; need(native_key(row)==required,'child returned another case')
        prior,audit=bundle(destination,repair=True)
        need(all(native_key(r)!=required for r in prior),'case already appended')
        atomic_text(str(destination)+'.audit.log',audit+audit_record(row,command))
        atomic_text(destination,render(prior+[row]))
    elif operation=='import-reference':
        destination=sys.argv[5]
        source_path=Path(filename)
        need(source_path.resolve()!=Path(destination).resolve(),'reference source must differ from destination')
        raw=source_path.read_bytes()
        need(raw and raw.endswith(b'\n'),'reference native CSV is empty/truncated')
        reader=csv.DictReader(io.StringIO(raw.decode('utf-8-sig')),strict=True)
        source_header=reader.fieldnames
        need(source_header is not None,'reference header missing')
        if source_header==columns:
            audit_path=Path(str(source_path)+'.audit.log')
            need(audit_path.is_file(),'clean reference requires its audit.log')
            audit_raw=audit_path.read_text()
            need(audit_raw and audit_raw.endswith('\n'),'reference audit empty/truncated')
            source_rows=[]
            for line in audit_raw.splitlines():
                entry=json.loads(line)
                need(entry.get('schema')=='cafbcr-result-audit-v1','reference audit schema changed')
                row=entry['row']
                need(entry['key']==native_key(row) and entry['row_sha256']==row_hash(row),
                     'reference audit key/digest differs')
                source_rows.append(row)
            need(render(source_rows)==raw.decode('utf-8'),'reference clean CSV differs from its audit')
            reader=source_rows
        else:
            need(len(source_header)==len(expected_header) and set(source_header)==set(expected_header),
                 'reference needs native70 or clean46 with audit.log')
        selected=[]; seen=set(); source_count=0
        for row in reader:
            source_count+=1
            need(set(row)==set(expected_header) and all(v not in (None,'') for v in row.values()),
                 'reference has missing/extra fields')
            cfg=json.loads(row['ScenarioConfig'])
            need(row['ScenarioSchema']==schema and identifier(cfg)==row['ScenarioId'],
                 'reference scenario identity differs')
            key=native_key(row)
            need(key not in seen,'duplicate reference case: '+key); seen.add(key)
            if key in plan: selected.append(row)
        need(selected,'reference has no cases matching the current plan')
        # Apply the SAME full native guard to every selected record before
        # reading or changing the destination. Other scenes/Runs are reported
        # as outside this plan, never silently counted as completed.
        with tempfile.TemporaryDirectory(prefix='cafbcr-scale-reference-') as folder:
            native=Path(folder)/'selected.csv'
            with native.open('w',newline='') as stream:
                writer=csv.DictWriter(stream,fieldnames=expected_header,lineterminator='\n')
                writer.writeheader(); writer.writerows(selected)
            checked,_=read(native)
        prior,audit=bundle(destination,repair=False)
        existing={native_key(row):row for row in prior}
        added=[]
        for row in checked:
            key=native_key(row)
            if key in existing:
                need(row==existing[key],'reference conflicts with an already saved row: '+key)
            else: added.append(row)
        source='import-reference:'+str(source_path.resolve())+':sha256='+hashlib.sha256(raw).hexdigest()
        if added:
            atomic_text(str(destination)+'.audit.log',audit+''.join(audit_record(row,source) for row in added))
            atomic_text(destination,render(prior+added))
        print('参考合并：源行='+str(source_count)+' 匹配='+str(len(checked))+' 新增='+str(len(added))
              +' 已有相同='+str(len(checked)-len(added))+' 计划外='+str(source_count-len(checked))+'；仿真=0；原生文件保持')
    elif operation=='import':
        destination=sys.argv[5]
        need(not Path(destination).exists() and not Path(str(destination)+'.audit.log').exists(),
             'import target already exists; preserve it and use another output path')
        rows,_=read(filename)
        need(rows,'native import needs at least one complete result row')
        source='import-native:'+str(Path(filename).resolve())+':sha256='+hashlib.sha256(Path(filename).read_bytes()).hexdigest()
        atomic_text(str(destination)+'.audit.log',''.join(audit_record(row,source) for row in rows))
        atomic_text(destination,render(rows))
        print('已导入='+str(len(rows))+'；仿真=0；原生文件保持')
    else: raise ValueError('unknown CSV operation')
except (ValueError,KeyError,TypeError,csv.Error,OSError) as e:
    print('CSV拒绝: '+str(e),file=sys.stderr); sys.exit(2)
PY
}


cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
MODE=plan
IMPORT_NATIVE=
for option in "$@"; do
    case $option in
        --import-native=*) MODE=import; IMPORT_NATIVE=${option#*=} ;;
        --import-reference=*) MODE=import-reference; IMPORT_NATIVE=${option#*=} ;;
        --plan) MODE=plan ;; --check) MODE=check ;; --no-build) BUILD_FIRST=0 ;;
        --run|--resume) MODE=run ;;
        -h|--help) echo '默认仅预览；--check --no-build零节点及保存数核查；--run/--resume --no-build运行或续跑；--import-reference=原生Speed.csv --no-build只合并匹配同Build参考，可重复执行；--import-native=原生规模.csv仅导入完整匹配计划到新目标。'; exit 0 ;;
        *) die "未知选项：$option" ;;
    esac
done
[[ $MAX_JOBS =~ ^[1-9][0-9]*$ ]] && ((MAX_JOBS<=20)) || die 'MAX_JOBS须为1~20（离线正式配置）'
case $CA_LOOP_POLICY in off|terminate-revisit|alternate-revisit) ;; *) die '未知CA_LOOP_POLICY' ;; esac
case $CA_PROXY_RENEWAL_POLICY in eager|ack-stagger|ack-batch|direct-feedback) ;; *) die '未知续约策略' ;; esac
case $CA_PROXY_RELAY_POLICY in legacy|ack-stagger) ;; *) die '未知中继协调策略' ;; esac
[[ $CA_PROXY_RELAY_POLICY == legacy || $CA_PROXY_RENEWAL_POLICY == direct-feedback ]] || die '中继协调组合须使用direct-feedback'
case $CA_CONTROL_UNICAST_POLICY in legacy|one-hop) ;; *) die '未知控制寻址策略' ;; esac
[[ $CA_DIRECT_CM_POLICY == legacy ]] || die '本采集合同仅采用直接CM legacy'
for algo in "${ALGORITHMS[@]}"; do
    case $algo in cafbcr|ca-qbcr|ca-bcr|olsr|qrcf|cfr-eng|mognn) ;;
        *) die "采集比较对象不支持：$algo" ;;
    esac
done
for n in "${SIZES[@]}"; do [[ $n =~ ^[1-9][0-9]*$ ]] || die 'SIZES须为正整数'; done
for run in "${RNG_RUNS[@]}"; do [[ $run =~ ^[1-9][0-9]*$ ]] || die 'RNG_RUNS须为正整数'; done
declare -A PLANNED=() PREVIEWS=() IDS=() COMMANDS=() LABELS=() CONTEXT=() JOB_DIR=() JOB_KEY=() DONE=()
declare -a ORDER=()
TOTAL=0
BUILD_ID=preview
if [[ $MODE != plan ]]; then
    export NS_LOG='' NS_GLOBAL_VALUE='' LC_ALL=C
    ulimit -c 0
    command -v python3 >/dev/null || die '需要系统Python3标准库，无需安装第三方包'
    if ((BUILD_FIRST)); then ./ns3 build ca-fbcr-test -j "$BUILD_JOBS"; fi
    BUILD_ID=$(build_id)
    WORK=$(mktemp -d)
    PLAN="$WORK/plan.tsv"
    : > "$PLAN"
    trap cleanup EXIT
    trap 'exit 130' INT
    trap 'exit 143' TERM
    trap 'exit 129' HUP
fi

for n in "${SIZES[@]}"; do
for algo in "${ALGORITHMS[@]}"; do
for speeds in "${SPEED_RANGES[@]}"; do
    [[ $speeds == *:* ]] || die "速度须为min:max：$speeds"
    IFS=: read -r minimum maximum <<< "$speeds"
    for sigma in "${SHADOW_SIGMAS[@]}"; do
        add_group "$algo" "$n" "$minimum" "$maximum" 4 linked 1 1 "$sigma"
    done
done; done; done

((TOTAL>0)) || die '计划不能为空'
if [[ $MODE == plan ]]; then
    echo "计划=$TOTAL；仅预览；正式批次由作者启动；CSV=$CSV_FILE" >&2
    exit 0
fi
[[ $(build_id) == "$BUILD_ID" ]] || die '预检期间构建改变'
if [[ $MODE == check ]]; then
    csv_tool validate "$CSV_FILE" "$PLAN" "$BUILD_ID" > "$WORK/existing"
    saved=$(wc -l < "$WORK/existing")
    echo "参数预检通过：计划=$TOTAL，独立配置=${#PREVIEWS[@]}，已保存=$saved，未完成=$((TOTAL-saved))，节点创建=0，仿真=0；CSV=$CSV_FILE"
    exit 0
fi
command -v setsid >/dev/null || die '缺少setsid'
exec 8>> "$GLOBAL_LOCK_FILE"
flock -n 8 || die '同主机已有共享锁批次；请分别启动，避免叠加并行'
STATUS_FILE="$CSV_FILE.status.tsv"
FAILURE_ROOT="$CSV_FILE.failures"
mkdir -p -- "$(dirname -- "$CSV_FILE")"
exec 9>> "$CSV_FILE.lock"
flock -n 9 || die '此CSV已有批次运行'
[[ -s $STATUS_FILE ]] || printf 'Time\tEvent\tAlgorithm\tNodes\tSpeed(m/s)\tAsymLevel\tShadowSigma(dB)\tAsymRangeProfile\tSeed\tRngRun\tCaseKey\tBuildId\tExitCode\n' > "$STATUS_FILE"
record_event BEGIN - -
if [[ $MODE == import || $MODE == import-reference ]]; then
    [[ -n $IMPORT_NATIVE && -f $IMPORT_NATIVE ]] || die '导入需要一个完整旧版原生CSV'
    csv_tool "$MODE" "$IMPORT_NATIVE" "$PLAN" "$BUILD_ID" "$CSV_FILE" || die '导入身份或数据不符；原生文件保持'
    record_event IMPORTED - 0
    exit 0
fi
csv_tool resume-keys "$CSV_FILE" "$PLAN" "$BUILD_ID" > "$WORK/existing" || die '已有CSV身份／计数／完整性不符'
while IFS= read -r key; do [[ -z $key ]] || DONE[$key]=1; done < "$WORK/existing"
completed=0 skipped=${#DONE[@]} submitted=0
set +m
echo "本组计划=$TOTAL；已保存=$skipped；本次待运行=$((TOTAL-skipped))；CSV=$CSV_FILE"
echo "全局滚动并行上限=$MAX_JOBS；同组优先，空位允许跨组补充"
for key in "${ORDER[@]}"; do
    if [[ ${DONE[$key]+yes} ]]; then
        echo "跳过 ${LABELS[$key]}"; continue
    fi
    while ((${#JOB_DIR[@]}>=MAX_JOBS)); do collect_one; done
    [[ $(build_id) == "$BUILD_ID" ]] || die '运行期间构建改变'
    task_dir=$(mktemp -d "$WORK/run-XXXXXXXX")
    printf -v output_arg '%q' "--resultsCsv=$task_dir/result.csv"
    printf '%s\n%s\n%s\n' "$key" "$BUILD_ID" "scratch/ca-fbcr-test ${COMMANDS[$key]} $output_arg" > "$task_dir/case.txt"
    record_event START "$key" -
    echo "启动 [已提交=$((submitted+1)) 待执行总数=$((TOTAL-skipped)) 并行=$((1+${#JOB_DIR[@]}))/$MAX_JOBS] ${LABELS[$key]}"
    submitted=$((submitted+1))
    setsid ./ns3 run --no-build --cwd "$task_dir" "scratch/ca-fbcr-test ${COMMANDS[$key]} $output_arg" \
        8>&- 9>&- > "$task_dir/stdout.log" 2> "$task_dir/stderr.log" &
    pid=$!
    JOB_DIR[$pid]=$task_dir JOB_KEY[$pid]=$key
done
while ((${#JOB_DIR[@]})); do collect_one; done
record_event COMPLETE - 0
echo "结束：计划=$TOTAL，完成=$completed，跳过=$skipped，并行上限=$MAX_JOBS；CSV=$CSV_FILE"
