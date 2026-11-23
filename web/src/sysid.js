const benchTrajectories = /*__TRAJECTORIES__*/;
const radiansPerTick = 2 * Math.PI / 4096;
let benchRun, benchLog, displayedBenchLog, benchBusy = false, benchCancelled = false;
const benchMessage = text => $("benchMessage").textContent = text;
async function benchCommand(args) {
  const result = await command("sysid", args);
  if (!result.ok) throw Error(result.message || "System identification unavailable");
  return result;
}
function plotBench(log) {
  displayedBenchLog=log;
  const canvas = $("benchPlot"), ctx = canvas.getContext("2d");
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  const entries = log.entries;
  if (!entries?.length) return;
  const prediction = log.predicted;
  const series = [entries.map(e => e.goal_position), entries.map(e => e.position)];
  if (prediction) series.push(prediction);
  const channel=$("benchChannel").value;
  if (channel!=="position") {
    const values=entries.map((e,i)=>channel==="current_raw"?log.diagnostics?.[i]?.current_raw:e[channel]);
    if (!values.every(Number.isFinite)) { benchMessage("Channel unavailable"); return; }
    series.splice(0,series.length,values);
  }
  const min = Math.min(...series.map(points => Math.min(...points)));
  const span = Math.max(0.01, ...series.map(points => Math.max(...points) - min));
  series.forEach((points, n) => {
    ctx.strokeStyle = ["#898989", "#287ab8", "#e76f32"][n];
    ctx.beginPath();
    points.forEach((q,i) => {
      const x = 30 + 640 * entries[i].timestamp / (entries.at(-1).timestamp || 1);
      const y = 150 - 130 * (q-min)/span;
      if (i) ctx.lineTo(x,y); else ctx.moveTo(x,y);
    });
    ctx.stroke();
  });
  entries.forEach((e,i)=>{
    if (i && e.torque_enable!==entries[i-1].torque_enable) {
      const x=30+640*e.timestamp/entries.at(-1).timestamp;
      ctx.fillStyle="#444"; ctx.fillText(e.torque_enable?"ON":"OFF",x,15);
      ctx.fillRect(x,20,1,130);
    }
  });
  ctx.fillText((min+span).toFixed(2),0,20); ctx.fillText(min.toFixed(2),0,150);
  ctx.fillText("0 s",30,170); ctx.fillText(entries.at(-1).timestamp.toFixed(2)+" s",640,170);
}
async function recordBench(directory) {
  try {
    const number = id => {
      const input=$(id);
      if (!Number.isFinite(input.valueAsNumber) || input.valueAsNumber<Number(input.min) || input.valueAsNumber>Number(input.max)) throw Error("Check bench settings");
      return input.valueAsNumber;
    };
    const id=number("benchId"), kp=number("benchGain"), zero=number("benchZero");
    const name=$("benchTrajectory").value, points=benchTrajectories[name];
    const limit=number("benchLimit") * Math.PI/180;
    if (points.some(([q]) => Math.abs(q*radiansPerTick)>limit || zero+q<0 || zero+q>4095))
      throw Error("Trajectory exceeds bench travel");
    const meta={mass:number("benchMass"),"arm-mass":number("benchArm"),length:number("benchLength"),vin:number("benchVin"),kp,trajectory:name};
    const {model}=await benchCommand("inspect "+id);
    const ticks=Math.floor(limit/radiansPerTick);
    const {run}=await benchCommand(`prepare ${id} ${points.length} 5000 ${kp} ${Math.max(0,zero-ticks)} ${Math.min(4095,zero+ticks)}`);
    benchRun=run; benchLog=null;
    for (let i=0;i<points.length;i+=12) {
      if (benchCancelled) throw Error("Cancelled before start");
      const chunk=points.slice(i,i+12).map(([q,on])=>[q+zero,on].join(",")).join(" ");
      await benchCommand(`upload ${run} ${i} ${chunk}`);
    }
    if (benchCancelled) throw Error("Cancelled before start");
    await benchCommand("start "+run);
    benchMessage("Recording…");
    let status;
    do {
      await new Promise(resolve=>setTimeout(resolve,300));
      status=await benchCommand("status "+run);
    } while (!status.done);
    const entries=[], diagnostics=[];
    while(entries.length<status.count) {
      const {rows}=await benchCommand(`read ${run} ${entries.length}`);
      for (const [cmd,t,target,on,pos,v,pwm,current,volts,temp] of rows) {
        entries.push({timestamp:t/1e6,goal_position:(target-zero)*radiansPerTick,torque_enable:!!on,
          position:(pos-zero)*radiansPerTick,speed:v*0.229*Math.PI/30,load:Math.max(-1,Math.min(1,pwm/885)),input_volts:volts/10,temp});
        diagnostics.push({command_us:cmd,current_raw:current});
      }
    }
    benchLog={...meta,motor:model,arm_mass:meta["arm-mass"],entries,diagnostics,result:status.message};
    plotBench(benchLog); benchMessage(status.message+" · "+entries.length+" samples");
    if (directory) {
      const file=await directory.getFileHandle(benchLog.result+"-"+name+"-"+kp+"-"+Date.now()+".json",{create:true});
      const writer=await file.createWritable();
      await writer.write(JSON.stringify(benchLog));
      await writer.close();
      await benchCommand("release "+run);
      benchRun=null;
      if (status.message!=="complete") throw Error(status.message);
    }
  } catch(e) { throw e; }
}
$("benchStart").onclick=async()=>{
  if (benchBusy || benchRun) return;
  benchBusy=true; benchCancelled=false;
  const controls=["benchTest","benchTrajectory","benchGain","benchId","benchZero","benchLimit","benchMass","benchArm","benchLength","benchVin"];
  const originalName=$("benchTrajectory").value, originalGain=$("benchGain").value;
  controls.forEach(id=>$(id).disabled=true);
  try {
    if ($("benchTest").value==="single") await recordBench();
    else {
      const directory=await window.showDirectoryPicker({mode:"readwrite"});
      for (const gain of [50,100,200,300,400]) {
        for (const name of ["sin_sin","lift_and_drop","up_and_down","sin_time_square"]) {
          if (benchCancelled) throw Error("Cancelled");
          $("benchGain").value=gain; $("benchTrajectory").value=name;
          await recordBench(directory);
          await new Promise(resolve=>setTimeout(resolve,3000));
        }
      }
    }
  } catch(e) { benchMessage(e.message); }
  finally {
    controls.forEach(id=>$(id).disabled=false);
    $("benchTrajectory").value=originalName; $("benchGain").value=originalGain; benchBusy=false;
  }
};
$("benchAbort").onclick=()=>{
  benchCancelled=true;
  if (benchRun) benchCommand("abort "+benchRun).catch(e=>benchMessage(e.message));
};
$("benchChannel").onchange=()=>displayedBenchLog && plotBench(displayedBenchLog);
$("benchSave").onclick=()=>{
  if (!benchLog) return;
  const url=URL.createObjectURL(new Blob([JSON.stringify(benchLog)],{type:"application/json"}));
  const a=document.createElement("a"); a.href=url; a.download=benchLog.result+"-"+benchLog.trajectory+"-"+Date.now()+".json"; a.click();
  setTimeout(()=>URL.revokeObjectURL(url),1000);
};
$("benchRelease").onclick=async()=>{
  if (benchBusy || !benchRun) return;
  try { await benchCommand("release "+benchRun); benchRun=null; benchMessage("Ready"); }
  catch(e){benchMessage(e.message);}
};
$("benchOpen").onchange=async e=>{
  try {const log=JSON.parse(await e.target.files[0].text()); plotBench(log); benchMessage(log.trajectory+" "+(log.mae_deg===undefined?"":"MAE "+log.mae_deg.toFixed(3)+"°"));}
  catch(err){benchMessage(err.message);}
};
