import { useState, useRef, useEffect, useMemo, useCallback } from "react";

/* ═══ RLE Preset Data (SLAM/DGIST/ROBOT) ═══ */
const PRESETS_RLE={"SLAM":[[4,-4,.0785,20],[5.367,1.633,.0601,56],[1.633,5.367,.0601,56],[0,0,.01,1],[4,-4,.0785,10],[5,5,.2,15],[4,-4,.0785,20],[5,5,.2,15],[-4,4,.0785,10],[5,5,.2,8],[0,0,.01,1],[4,-4,.0818,8],[5,5,.2067,15],[-4,4,.077,17],[5,5,.2067,15],[4,-4,.0785,20],[5,5,.22,7],[-4,4,.0764,12],[5,5,.2143,7],[0,0,.01,1],[4,-4,.0785,10],[5,5,.2,15],[-4,4,.0791,16],[5,5,.2,9],[4,-4,.08,12],[5,5,.2,9],[-4,4,.0791,16],[5,5,.2,15]],"DGIST":[[4,-4,.0785,10],[5,5,.2,15],[2.567,4.433,.0601,112],[0,0,.01,1],[5,5,.2,8],[4,-4,.0785,10],[5,5,.2,15],[4,-4,.0785,10],[5,5,.2,8],[4,-4,.0785,10],[5,5,.2143,7],[-4,4,.0785,10],[5,5,.2,4],[0,0,.01,1],[4,-4,.0785,10],[5,5,.2,15],[0,0,.01,1],[4,-4,.0785,20],[5.367,1.633,.0601,56],[1.633,5.367,.0601,56],[0,0,.01,1],[5,5,.2,5],[4,-4,.0785,10],[5,5,.2,15],[4,-4,.0785,10],[5,5,.2,5],[4,-4,.0785,20],[5,5,.2,10]],"ROBOT":[[4,-4,.0785,10],[5,5,.2,15],[1.633,5.367,.0601,56],[-4,4,.0764,4],[5,5,.2125,8],[0,0,.01,1],[4,-4,.0785,10],[5,5,.2,15],[-4,4,.0785,10],[5,5,.2,8],[-4,4,.0785,10],[5,5,.2,15],[-4,4,.0785,10],[5,5,.2,8],[0,0,.01,1],[4,-4,.0785,10],[5,5,.2,15],[1.633,5.367,.0601,112],[0,0,.01,1],[4,-4,.0785,10],[5,5,.2,15],[-4,4,.0785,10],[5,5,.2,8],[-4,4,.0785,10],[5,5,.2,15],[-4,4,.0785,10],[5,5,.2,8],[0,0,.01,1],[5,5,.2,5],[4,-4,.0785,10],[5,5,.2,15],[4,-4,.0785,10],[5,5,.2,5],[4,-4,.0785,20],[5,5,.2,10]]};

function decodeRLE(rle){const out=[];for(const[pR,pL,dt,n]of rle)for(let i=0;i<n;i++)out.push({pR,pL,dt});return out}

const C={bg:"#0a0f1a",panel:"#111827",bd:"#1e293b",grid:"#1a2332",gAxis:"#2a3a52",
acc:"#00d4aa",accD:"#00d4aa44",org:"#ff8c42",pnk:"#ff5ca0",blu:"#4da6ff",yel:"#fbbf24",
txt:"#e2e8f0",dim:"#64748b",mut:"#475569",mBg:"#0d1520",mBd:"#1e3a5f",
rBg:"#0a1a12",rBd:"#0d503a",wBg:"#1a1000",wBd:"#5a3d00"};

const dtr=d=>d*Math.PI/180;
function composeSE2(a,b){const c=Math.cos(dtr(a.th)),s=Math.sin(dtr(a.th));return{x:a.x+c*b.x-s*b.y,y:a.y+s*b.x+c*b.y,th:((a.th+b.th)%360+360)%360}}
function fN(n){return(Math.abs(n)<.005?0:n).toFixed(2)}
function fA(n){return(Math.abs(n)<.05?0:n).toFixed(1)}

function stepToSE2(pR,pL,dt,r,ell){
  const v=r*(pR+pL)/2, w=r*(pR-pL)/(2*ell);
  if(Math.abs(w)<1e-7)return{x:v*dt,y:0,th:0};
  const dth=w*dt;return{x:(v/w)*Math.sin(dth),y:(v/w)*(1-Math.cos(dth)),th:dth*180/Math.PI};
}

function simulate(steps,r,ell){
  let cur={x:0,y:0,th:0};const trail=[{...cur}];
  for(const s of steps){const rel=stepToSE2(s.pR,s.pL,s.dt,r,ell);cur=composeSE2(cur,rel);trail.push({...cur})}
  return trail;
}

/* ═══ Canvas with hover tooltip ═══ */
function TraceCanvas({trail,animIdx,W=500,H=500}){
  const ref=useRef(null);
  const [hover,setHover]=useState(null); // {idx, cx, cy} canvas-pixel coords of hovered point
  // Store transform params so mousemove can reuse them
  const xformRef=useRef(null);

  // Precompute canvas positions for every trail point (for hit-testing)
  const canvasPts=useMemo(()=>{
    if(trail.length<2)return[];
    let mnX=1e9,mxX=-1e9,mnY=1e9,mxY=-1e9;
    for(const t of trail){mnX=Math.min(mnX,t.x);mxX=Math.max(mxX,t.x);mnY=Math.min(mnY,t.y);mxY=Math.max(mxY,t.y)}
    const pad=.8;mnX-=pad;mxX+=pad;mnY-=pad;mxY+=pad;
    const rX=mxX-mnX||1,rY=mxY-mnY||1,sc=Math.min((W-50)/rX,(H-50)/rY);
    const ox=25+(W-50-rX*sc)/2,oy=25+(H-50-rY*sc)/2;
    xformRef.current={mnX,mxX,mnY,mxY,rX,rY,sc,ox,oy};
    return trail.map(t=>({px:ox+(t.x-mnX)*sc, py:oy+(mxY-t.y)*sc}));
  },[trail,W,H]);

  const handleMouseMove=useCallback((e)=>{
    const cv=ref.current;if(!cv||canvasPts.length<2)return;
    const rect=cv.getBoundingClientRect();
    const mx=e.clientX-rect.left, my=e.clientY-rect.top;
    // Find nearest trail point within 15px
    let bestD=225,bestI=-1; // 15^2
    // Sample every few points for performance on large trails
    const step=Math.max(1,Math.floor(canvasPts.length/1000));
    for(let i=0;i<canvasPts.length;i+=step){
      const dx=canvasPts[i].px-mx,dy=canvasPts[i].py-my;
      const d2=dx*dx+dy*dy;
      if(d2<bestD){bestD=d2;bestI=i}
    }
    // Refine around best
    if(bestI>=0){
      const lo=Math.max(0,bestI-step),hi=Math.min(canvasPts.length-1,bestI+step);
      for(let i=lo;i<=hi;i++){
        const dx=canvasPts[i].px-mx,dy=canvasPts[i].py-my;
        const d2=dx*dx+dy*dy;
        if(d2<bestD){bestD=d2;bestI=i}
      }
    }
    if(bestI>=0)setHover({idx:bestI,cx:canvasPts[bestI].px,cy:canvasPts[bestI].py});
    else setHover(null);
  },[canvasPts]);

  const handleMouseLeave=useCallback(()=>setHover(null),[]);

  useEffect(()=>{
    const cv=ref.current;if(!cv)return;const ctx=cv.getContext("2d");
    const dpr=window.devicePixelRatio||1;cv.width=W*dpr;cv.height=H*dpr;ctx.scale(dpr,dpr);
    let mnX=1e9,mxX=-1e9,mnY=1e9,mxY=-1e9;
    const pts=animIdx>=0?trail.slice(0,animIdx+1):trail;
    for(const t of trail){mnX=Math.min(mnX,t.x);mxX=Math.max(mxX,t.x);mnY=Math.min(mnY,t.y);mxY=Math.max(mxY,t.y)}
    const pad=.8;mnX-=pad;mxX+=pad;mnY-=pad;mxY+=pad;
    const rX=mxX-mnX||1,rY=mxY-mnY||1,sc=Math.min((W-50)/rX,(H-50)/rY);
    const ox=25+(W-50-rX*sc)/2,oy=25+(H-50-rY*sc)/2;
    const tc=(x,y)=>[ox+(x-mnX)*sc,oy+(mxY-y)*sc];

    ctx.fillStyle=C.bg;ctx.fillRect(0,0,W,H);
    // Compute nice tick step: target ~6-10 ticks on the longer axis
    const maxRange=Math.max(rX,rY);
    const rawStep=maxRange/8;
    const mag=Math.pow(10,Math.floor(Math.log10(rawStep)));
    const residual=rawStep/mag;
    const niceStep=residual<=1.5?mag:residual<=3?2*mag:residual<=7?5*mag:10*mag;
    // Grid lines + tick labels
    ctx.font="11px monospace";
    // Vertical grid (X ticks)
    for(let gx=Math.ceil(mnX/niceStep)*niceStep;gx<=mxX;gx+=niceStep){
      const[px]=tc(gx,0);
      ctx.strokeStyle=C.grid;ctx.lineWidth=.5;
      ctx.beginPath();ctx.moveTo(px,0);ctx.lineTo(px,H);ctx.stroke();
      // Tick label at bottom
      const label=Math.abs(gx)<1e-9?"0":niceStep>=1?gx.toFixed(0):gx.toFixed(1);
      ctx.fillStyle=C.dim;ctx.textAlign="center";
      ctx.fillText(label,px,H-4);
    }
    // Horizontal grid (Y ticks)
    for(let gy=Math.ceil(mnY/niceStep)*niceStep;gy<=mxY;gy+=niceStep){
      const[,py]=tc(0,gy);
      ctx.strokeStyle=C.grid;ctx.lineWidth=.5;
      ctx.beginPath();ctx.moveTo(0,py);ctx.lineTo(W,py);ctx.stroke();
      // Tick label at left
      const label=Math.abs(gy)<1e-9?"0":niceStep>=1?gy.toFixed(0):gy.toFixed(1);
      ctx.fillStyle=C.dim;ctx.textAlign="left";
      ctx.fillText(label,3,py-3);
    }
    // Axes
    ctx.strokeStyle=C.gAxis;ctx.lineWidth=1;
    if(mnX<=0&&mxX>=0){const[px]=tc(0,0);ctx.beginPath();ctx.moveTo(px,0);ctx.lineTo(px,H);ctx.stroke()}
    if(mnY<=0&&mxY>=0){const[,py]=tc(0,0);ctx.beginPath();ctx.moveTo(0,py);ctx.lineTo(W,py);ctx.stroke()}
    // Origin
    if(mnX<=0&&mxX>=0&&mnY<=0&&mxY>=0){const[ox2,oy2]=tc(0,0);ctx.fillStyle=C.mut;ctx.beginPath();ctx.arc(ox2,oy2,4,0,2*Math.PI);ctx.fill();ctx.fillStyle=C.dim;ctx.font="bold 12px monospace";ctx.textAlign="left";ctx.fillText("W",ox2+6,oy2-5)}
    // Ghost trail (full, faint)
    if(animIdx>=0&&trail.length>1){ctx.strokeStyle=C.acc+"22";ctx.lineWidth=1.5;ctx.beginPath();const[sx,sy]=tc(trail[0].x,trail[0].y);ctx.moveTo(sx,sy);for(let i=1;i<trail.length;i++){const[tx,ty]=tc(trail[i].x,trail[i].y);ctx.lineTo(tx,ty)}ctx.stroke()}
    // Active trail
    if(pts.length>1){ctx.strokeStyle=C.acc;ctx.lineWidth=2.5;ctx.lineJoin="round";ctx.lineCap="round";ctx.beginPath();const[sx,sy]=tc(pts[0].x,pts[0].y);ctx.moveTo(sx,sy);for(let i=1;i<pts.length;i++){const[tx,ty]=tc(pts[i].x,pts[i].y);ctx.lineTo(tx,ty)}ctx.stroke()}
    // Start
    if(pts.length>0){const[sx,sy]=tc(pts[0].x,pts[0].y);ctx.fillStyle=C.org;ctx.beginPath();ctx.arc(sx,sy,5,0,2*Math.PI);ctx.fill()}
    // Robot
    if(pts.length>1){const f=pts[pts.length-1];const[cx,cy]=tc(f.x,f.y);const rd=dtr(f.th);const sz=12;
    ctx.save();ctx.translate(cx,cy);ctx.rotate(-rd);ctx.fillStyle=C.acc+"55";ctx.strokeStyle=C.acc;ctx.lineWidth=2;ctx.beginPath();ctx.moveTo(sz,0);ctx.lineTo(-sz*.7,-sz*.65);ctx.lineTo(-sz*.7,sz*.65);ctx.closePath();ctx.fill();ctx.stroke();
    ctx.fillStyle=C.yel;ctx.fillRect(-sz*.4,-sz*.8,sz*.5,3);ctx.fillRect(-sz*.4,sz*.8-3,sz*.5,3);ctx.restore()}
    // Info
    ctx.fillStyle=C.dim;ctx.font="12px monospace";ctx.textAlign="right";
    const cur=pts[pts.length-1]||{x:0,y:0,th:0};
    ctx.fillText(`step ${pts.length-1}/${trail.length-1}`,W-8,H-8);
    ctx.fillText(`x=${fN(cur.x)} y=${fN(cur.y)} θ=${fA(cur.th)}°`,W-8,H-22);

    // Hover highlight
    if(hover&&hover.idx<trail.length){
      const hp=trail[hover.idx];
      const[hx,hy]=tc(hp.x,hp.y);
      // Crosshair
      ctx.strokeStyle=C.yel+"88";ctx.lineWidth=1;ctx.setLineDash([3,3]);
      ctx.beginPath();ctx.moveTo(hx,0);ctx.lineTo(hx,H);ctx.stroke();
      ctx.beginPath();ctx.moveTo(0,hy);ctx.lineTo(W,hy);ctx.stroke();
      ctx.setLineDash([]);
      // Dot
      ctx.fillStyle=C.yel;ctx.beginPath();ctx.arc(hx,hy,6,0,2*Math.PI);ctx.fill();
      ctx.strokeStyle=C.bg;ctx.lineWidth=2;ctx.beginPath();ctx.arc(hx,hy,6,0,2*Math.PI);ctx.stroke();
      // Tooltip box
      const txt1=`step ${hover.idx}`;
      const txt2=`x = ${fN(hp.x)}`;
      const txt3=`y = ${fN(hp.y)}`;
      const txt4=`θ = ${fA(hp.th)}°`;
      ctx.font="bold 12px monospace";
      const tw=Math.max(ctx.measureText(txt1).width,ctx.measureText(txt2).width,ctx.measureText(txt3).width,ctx.measureText(txt4).width)+16;
      const th=68;
      // Position tooltip so it doesn't go off-canvas
      let tx2=hx+14,ty2=hy-th-8;
      if(tx2+tw>W-4)tx2=hx-tw-14;
      if(ty2<4)ty2=hy+14;
      // Box shadow
      ctx.fillStyle="#000000aa";
      ctx.beginPath();ctx.roundRect(tx2+2,ty2+2,tw,th,6);ctx.fill();
      // Box
      ctx.fillStyle=C.panel+"ee";
      ctx.beginPath();ctx.roundRect(tx2,ty2,tw,th,6);ctx.fill();
      ctx.strokeStyle=C.yel+"88";ctx.lineWidth=1;
      ctx.beginPath();ctx.roundRect(tx2,ty2,tw,th,6);ctx.stroke();
      // Text
      ctx.textAlign="left";
      ctx.fillStyle=C.yel;ctx.fillText(txt1,tx2+8,ty2+16);
      ctx.fillStyle=C.acc;ctx.font="12px monospace";
      ctx.fillText(txt2,tx2+8,ty2+32);
      ctx.fillText(txt3,tx2+8,ty2+46);
      ctx.fillText(txt4,tx2+8,ty2+60);
    }
  },[trail,animIdx,W,H,hover]);

  return(
    <div style={{position:"relative",display:"inline-block"}}>
      <canvas ref={ref}
        onMouseMove={handleMouseMove} onMouseLeave={handleMouseLeave}
        style={{width:W,height:H,borderRadius:12,border:`1px solid ${C.bd}`,cursor:hover?"crosshair":"default"}}/>
    </div>
  );
}

/* ═══ Btn helper ═══ */
const Btn=({onClick,disabled,children,color=C.acc,style={}})=>(
  <button onClick={onClick} disabled={disabled} style={{
    background:disabled?C.bd:color+"22",color:disabled?C.mut:color,
    border:`1px solid ${disabled?C.bd:color+"55"}`,borderRadius:8,padding:"6px 14px",
    cursor:disabled?"not-allowed":"pointer",fontSize:12,fontWeight:600,fontFamily:"monospace",...style
  }}>{children}</button>
);

/* ═══ Main ═══ */
export default function App(){
  const [mode,setMode]=useState("wheel"); // "wheel" | "twist" | "preset" | "interactive"
  const [rParam,setR]=useState(0.1);
  const [lParam,setL]=useState(0.2);

  // Wheel-level steps: [{pR, pL, dt}]
  const [wSteps,setWSteps]=useState([{pR:2,pL:1.5,dt:1},{pR:1,pL:2,dt:1}]);
  // Twist-level steps: [{v, w, dt}]
  const [tSteps,setTSteps]=useState([{v:0.5,w:0,dt:2},{v:0.3,w:0.5,dt:3}]);
  // Preset
  const [preset,setPreset]=useState(null);
  const [presetSteps,setPresetSteps]=useState([]);

  // Interactive keyboard mode
  const [iSteps,setISteps]=useState([]);
  const [liveV,setLiveV]=useState(0);
  const [liveW,setLiveW]=useState(0);
  const [iDriving,setIDriving]=useState(false);
  const keysRef=useRef(new Set());
  const iLoopRef=useRef(null);
  const LIVE_DT=0.05; // time per tick
  const V_SPEED=0.5;  // m/s per arrow key
  const W_SPEED=1.5;  // rad/s per arrow key

  // Keyboard listeners (always mounted, only act when interactive+driving)
  useEffect(()=>{
    const onDown=(e)=>{
      if(mode!=="interactive"||!iDriving)return;
      if(["ArrowUp","ArrowDown","ArrowLeft","ArrowRight"].includes(e.key)){
        e.preventDefault();keysRef.current.add(e.key);
      }
    };
    const onUp=(e)=>{
      keysRef.current.delete(e.key);
    };
    window.addEventListener("keydown",onDown);
    window.addEventListener("keyup",onUp);
    return()=>{window.removeEventListener("keydown",onDown);window.removeEventListener("keyup",onUp)};
  },[mode,iDriving]);

  // Game loop: emit steps while keys are held
  useEffect(()=>{
    if(mode!=="interactive"||!iDriving){
      if(iLoopRef.current){clearInterval(iLoopRef.current);iLoopRef.current=null}
      setLiveV(0);setLiveW(0);return;
    }
    iLoopRef.current=setInterval(()=>{
      const keys=keysRef.current;
      let v=0,w=0;
      if(keys.has("ArrowUp"))v+=V_SPEED;
      if(keys.has("ArrowDown"))v-=V_SPEED;
      if(keys.has("ArrowLeft"))w+=W_SPEED;
      if(keys.has("ArrowRight"))w-=W_SPEED;
      setLiveV(v);setLiveW(w);
      if(v!==0||w!==0){
        const wh=twistToWheel(v,w);
        setISteps(prev=>[...prev,{pR:wh.pR,pL:wh.pL,dt:LIVE_DT}]);
      }
    },50); // 20 Hz
    return()=>{if(iLoopRef.current)clearInterval(iLoopRef.current)};
  },[mode,iDriving,rParam,lParam]);

  // Animation
  const [animIdx,setAnimIdx]=useState(-1);
  const [playing,setPlaying]=useState(false);
  const animRef=useRef(null);

  // Convert twist to wheel
  const twistToWheel=(v,w)=>({pR:(v+w*lParam)/rParam,pL:(v-w*lParam)/rParam});

  // Active steps as [{pR,pL,dt}]
  const activeSteps=useMemo(()=>{
    if(mode==="preset") return presetSteps;
    if(mode==="twist") return tSteps.map(s=>{const wh=twistToWheel(s.v,s.w);return{pR:wh.pR,pL:wh.pL,dt:s.dt}});
    if(mode==="interactive") return iSteps;
    return wSteps;
  },[mode,wSteps,tSteps,presetSteps,iSteps,rParam,lParam]);

  const trail=useMemo(()=>simulate(activeSteps,rParam,lParam),[activeSteps,rParam,lParam]);

  // Animation
  const startAnim=useCallback(()=>{setAnimIdx(0);setPlaying(true)},[]);
  const stopAnim=useCallback(()=>{setPlaying(false);if(animRef.current)cancelAnimationFrame(animRef.current)},[]);
  const resetAnim=useCallback(()=>{stopAnim();setAnimIdx(-1)},[stopAnim]);

  useEffect(()=>{
    if(!playing)return;
    const maxIdx=trail.length-1;
    const speed=Math.max(1,Math.floor(maxIdx/300));
    let frame;
    const step=()=>{
      setAnimIdx(prev=>{
        if(prev>=maxIdx){setPlaying(false);return maxIdx}
        return Math.min(prev+speed,maxIdx);
      });
      frame=requestAnimationFrame(step);
    };
    frame=requestAnimationFrame(step);
    animRef.current=frame;
    return()=>cancelAnimationFrame(frame);
  },[playing,trail.length]);

  // ─── Interactive keyboard mode ───
  useEffect(()=>{
    if(mode!=="interactive")return;
    const onDown=(e)=>{
      if(["ArrowUp","ArrowDown","ArrowLeft","ArrowRight"].includes(e.key)){
        e.preventDefault();keysRef.current.add(e.key);
      }
    };
    const onUp=(e)=>{keysRef.current.delete(e.key)};
    window.addEventListener("keydown",onDown);
    window.addEventListener("keyup",onUp);

    const tick=()=>{
      const keys=keysRef.current;
      let v=0,w=0;
      if(keys.has("ArrowUp")) v+=V_STEP;
      if(keys.has("ArrowDown")) v-=V_STEP;
      if(keys.has("ArrowLeft")) w+=W_STEP;
      if(keys.has("ArrowRight")) w-=W_STEP;
      setLiveV(v);setLiveW(w);
      if(v!==0||w!==0){
        const wh=twistToWheel(v,w);
        setISteps(prev=>[...prev,{pR:wh.pR,pL:wh.pL,dt:LIVE_DT}]);
      }
    };
    const iv=setInterval(tick,50); // 20 Hz
    iLoopRef.current=iv;
    return()=>{clearInterval(iv);window.removeEventListener("keydown",onDown);window.removeEventListener("keyup",onUp);keysRef.current.clear();setLiveV(0);setLiveW(0)};
  },[mode,rParam,lParam]);

  const loadPreset=(name)=>{
    const steps=decodeRLE(PRESETS_RLE[name]);
    setPresetSteps(steps);setPreset(name);setMode("preset");resetAnim();
  };

  // Add forms
  const [newW,setNewW]=useState({pR:"5",pL:"5",dt:"1"});
  const [newT,setNewT]=useState({v:"0.5",w:"0",dt:"2"});

  const addWStep=()=>{setWSteps(p=>[...p,{pR:+newW.pR,pL:+newW.pL,dt:+newW.dt}]);resetAnim()};
  const addTStep=()=>{setTSteps(p=>[...p,{v:+newT.v,w:+newT.w,dt:+newT.dt}]);resetAnim()};

  const finalPose=trail[trail.length-1]||{x:0,y:0,th:0};
  const totalTime=activeSteps.reduce((s,c)=>s+c.dt,0);

  const S={input:{width:56,background:C.panel,color:C.txt,border:`1px solid ${C.bd}`,borderRadius:5,padding:"4px 6px",fontSize:12,fontFamily:"monospace"},
    label:{color:C.dim,fontSize:12,fontFamily:"monospace"},
    section:{background:C.panel,border:`1px solid ${C.bd}`,borderRadius:10,padding:"10px 12px"}};

  return(
    <div style={{minHeight:"100vh",background:C.bg,color:C.txt,fontFamily:"system-ui,sans-serif",padding:16}}>
      {/* Header */}
      <div style={{textAlign:"center",marginBottom:14}}>
        <div style={{fontSize:11,color:C.acc,textTransform:"uppercase",letterSpacing:3,fontFamily:"monospace"}}>RT604 SLAM Course · Interactive Lab</div>
        <h1 style={{fontSize:24,fontWeight:800,margin:"4px 0",background:`linear-gradient(135deg,${C.acc},${C.blu})`,WebkitBackgroundClip:"text",WebkitTextFillColor:"transparent"}}>
          SE(2) Pose Composition
        </h1>
        <p style={{color:C.dim,fontSize:13,margin:0,fontFamily:"monospace"}}>Differential Drive · Nonholonomic Constraint · Unlimited Steps</p>
      </div>

      {/* Robot params */}
      <div style={{display:"flex",justifyContent:"center",gap:16,marginBottom:10,flexWrap:"wrap",alignItems:"center"}}>
        <span style={S.label}>Robot Params:</span>
        {[{l:"r (wheel radius)",v:rParam,set:setR,min:.02,max:.5,step:.01},
          {l:"l (half baseline)",v:lParam,set:setL,min:.05,max:1,step:.01}].map(p=>(
          <label key={p.l} style={{display:"flex",alignItems:"center",gap:4,...S.label}}>
            {p.l}
            <input type="number" step={p.step} min={p.min} max={p.max} value={p.v}
              onChange={e=>{p.set(Math.max(p.min,+e.target.value));resetAnim()}}
              style={{...S.input,width:64}}/>
            <span style={{color:C.txt}}>m</span>
          </label>
        ))}
      </div>

      {/* Mode tabs */}
      <div style={{display:"flex",justifyContent:"center",gap:6,marginBottom:12,flexWrap:"wrap"}}>
        {[{id:"wheel",label:"Low-Level (φ_R, φ_L)"},{id:"twist",label:"High-Level (v, ω)"},{id:"interactive",label:"⌨ Keyboard Drive"},{id:"preset",label:"Presets"}].map(m=>(
          <Btn key={m.id} onClick={()=>{setMode(m.id);setIDriving(false);resetAnim()}} color={mode===m.id?C.acc:C.dim} style={{background:mode===m.id?C.acc+"22":C.panel}}>{m.label}</Btn>
        ))}
      </div>

      {/* Preset buttons */}
      {mode==="preset"&&(
        <div style={{display:"flex",justifyContent:"center",gap:8,marginBottom:12}}>
          {Object.keys(PRESETS_RLE).map(name=>(
            <Btn key={name} onClick={()=>loadPreset(name)} color={preset===name?C.acc:C.org}
              style={{fontSize:14,fontWeight:800,letterSpacing:2,padding:"8px 20px",background:preset===name?C.acc+"33":C.panel}}>
              {name} <span style={{fontSize:10,opacity:.6}}>({decodeRLE(PRESETS_RLE[name]).length})</span>
            </Btn>
          ))}
        </div>
      )}

      <div style={{display:"flex",gap:16,maxWidth:1200,margin:"0 auto",flexWrap:"wrap",justifyContent:"center"}}>
        {/* Left: Canvas + animation + info */}
        <div style={{display:"flex",flexDirection:"column",gap:10}}>
          <TraceCanvas trail={trail} animIdx={animIdx} W={520} H={520}/>

          {/* Animation controls */}
          <div style={{display:"flex",gap:6,justifyContent:"center",alignItems:"center"}}>
            <Btn onClick={startAnim} disabled={playing} color={C.acc}>▶ Animate</Btn>
            <Btn onClick={stopAnim} disabled={!playing} color={C.pnk}>⏸ Pause</Btn>
            <Btn onClick={resetAnim} color={C.dim}>⏹ Reset View</Btn>
            {animIdx>=0&&(
              <div style={{marginLeft:8}}>
                <input type="range" min={0} max={trail.length-1} value={animIdx>=0?animIdx:trail.length-1}
                  onChange={e=>{stopAnim();setAnimIdx(+e.target.value)}}
                  style={{width:160,accentColor:C.acc}}/>
              </div>
            )}
          </div>

          {/* Pose display */}
          <div style={{...S.section,display:"flex",gap:16,flexWrap:"wrap",alignItems:"center"}}>
            <div style={{fontFamily:"monospace",fontSize:13}}>
              <div style={{color:C.dim,fontSize:11,marginBottom:4}}>Current Pose (step {animIdx>=0?animIdx:trail.length-1}/{trail.length-1})</div>
              <div><span style={{color:C.blu}}>x</span> = <span style={{color:C.acc,fontWeight:700}}>{fN((animIdx>=0?trail[animIdx]:finalPose).x)}</span> m</div>
              <div><span style={{color:C.blu}}>y</span> = <span style={{color:C.acc,fontWeight:700}}>{fN((animIdx>=0?trail[animIdx]:finalPose).y)}</span> m</div>
              <div><span style={{color:C.blu}}>θ</span> = <span style={{color:C.acc,fontWeight:700}}>{fA((animIdx>=0?trail[animIdx]:finalPose).th)}°</span></div>
            </div>
            <div style={{fontFamily:"monospace",fontSize:12,color:C.dim}}>
              <div>Total steps: <span style={{color:C.txt}}>{activeSteps.length}</span></div>
              <div>Total time: <span style={{color:C.txt}}>{totalTime.toFixed(2)}s</span></div>
              <div>r={rParam}m, l={lParam}m</div>
            </div>
            {/* 3x3 matrix */}
            <div style={{fontFamily:"monospace",fontSize:11,background:C.mBg,border:`1px solid ${C.mBd}`,borderRadius:8,padding:"6px 8px"}}>
              <div style={{color:C.acc,fontWeight:700,fontSize:12,marginBottom:3}}>T_world</div>
              {(()=>{const p=animIdx>=0?trail[animIdx]:finalPose;const co=Math.cos(dtr(p.th)),si=Math.sin(dtr(p.th));
                return[[fN(co),fN(-si),fN(p.x)],[fN(si),fN(co),fN(p.y)],["0.00","0.00","1.00"]].map((row,ri)=>(
                  <div key={ri} style={{display:"flex",gap:6}}>{row.map((v,ci)=>(
                    <span key={ci} style={{width:40,textAlign:"right",color:ri===2?C.mut:ci===2?C.blu:C.txt,opacity:ri===2?.4:1}}>{v}</span>
                  ))}</div>
                ))})()}
            </div>
          </div>

          {/* Constraint info */}
          <div style={{background:C.wBg,border:`1px solid ${C.wBd}`,borderRadius:10,padding:"8px 12px",fontFamily:"monospace",fontSize:12,lineHeight:1.5,maxWidth:520}}>
            <span style={{color:C.yel,fontWeight:700}}>Nonholonomic Constraint: </span>
            <span style={{color:C.dim}}>ẏ_local = 0. The robot cannot slide sideways. v = r(φR+φL)/2, ω = r(φR-φL)/(2l).</span>
          </div>
        </div>

        {/* Right panel */}
        <div style={{display:"flex",flexDirection:"column",gap:10,minWidth:340,maxWidth:400,flex:1}}>
          {/* Wheel-level input */}
          {mode==="wheel"&&(<>
            <div style={S.section}>
              <div style={{color:C.org,fontWeight:700,fontSize:14,fontFamily:"monospace",marginBottom:8}}>Low-Level: Wheel Speeds</div>
              <div style={{color:C.dim,fontSize:12,fontFamily:"monospace",marginBottom:8}}>
                Specify right/left wheel angular velocities (rad/s) and duration.
              </div>
              <div style={{display:"flex",gap:6,alignItems:"center",flexWrap:"wrap",marginBottom:8}}>
                {[{l:"φ_R",k:"pR",c:C.org},{l:"φ_L",k:"pL",c:C.pnk},{l:"dt(s)",k:"dt",c:C.blu}].map(f=>(
                  <label key={f.k} style={{display:"flex",alignItems:"center",gap:3,color:f.c,fontSize:12,fontFamily:"monospace"}}>
                    {f.l}
                    <input type="number" step="0.1" value={newW[f.k]} onChange={e=>setNewW(p=>({...p,[f.k]:e.target.value}))} style={S.input}/>
                  </label>
                ))}
                <Btn onClick={addWStep} color={C.acc}>+ Add</Btn>
              </div>
              <div style={{display:"flex",gap:4}}>
                <Btn onClick={()=>{setWSteps([]);resetAnim()}} color={C.pnk}>Clear All</Btn>
                <Btn onClick={()=>{setWSteps([{pR:2,pL:1.5,dt:1},{pR:1,pL:2,dt:1}]);resetAnim()}} color={C.dim}>Reset Default</Btn>
              </div>
            </div>
            {/* Step list */}
            <div style={{...S.section,maxHeight:320,overflowY:"auto"}}>
              <div style={{color:C.dim,fontSize:11,fontFamily:"monospace",marginBottom:4}}>Steps ({wSteps.length})</div>
              {wSteps.length===0?<div style={{color:C.mut,fontSize:12,textAlign:"center",padding:8}}>No steps added yet.</div>:
              wSteps.map((s,i)=>(
                <div key={i} style={{display:"flex",gap:6,alignItems:"center",fontSize:12,fontFamily:"monospace",padding:"3px 4px",borderBottom:`1px solid ${C.bd}`}}>
                  <span style={{color:C.dim,width:28}}>{i+1}</span>
                  <span style={{color:C.org}}>φR={s.pR.toFixed(1)}</span>
                  <span style={{color:C.pnk}}>φL={s.pL.toFixed(1)}</span>
                  <span style={{color:C.blu}}>dt={s.dt.toFixed(2)}</span>
                  <button onClick={()=>{setWSteps(p=>p.filter((_,j)=>j!==i));resetAnim()}} style={{background:"none",border:"none",color:C.mut,cursor:"pointer",fontSize:12}}>×</button>
                </div>
              ))}
            </div>
          </>)}

          {/* Twist-level input */}
          {mode==="twist"&&(<>
            <div style={S.section}>
              <div style={{color:C.blu,fontWeight:700,fontSize:14,fontFamily:"monospace",marginBottom:8}}>High-Level: Body Twist (v, ω)</div>
              <div style={{color:C.dim,fontSize:12,fontFamily:"monospace",marginBottom:6}}>
                Specify linear velocity v (m/s), angular velocity ω (rad/s), and duration.
                Wheel speeds are computed as: φ_R = (v + ωl)/r, φ_L = (v - ωl)/r.
              </div>
              <div style={{display:"flex",gap:6,alignItems:"center",flexWrap:"wrap",marginBottom:8}}>
                {[{l:"v (m/s)",k:"v",c:C.acc},{l:"ω (rad/s)",k:"w",c:C.yel},{l:"dt (s)",k:"dt",c:C.blu}].map(f=>(
                  <label key={f.k} style={{display:"flex",alignItems:"center",gap:3,color:f.c,fontSize:12,fontFamily:"monospace"}}>
                    {f.l}
                    <input type="number" step="0.1" value={newT[f.k]} onChange={e=>setNewT(p=>({...p,[f.k]:e.target.value}))} style={S.input}/>
                  </label>
                ))}
                <Btn onClick={addTStep} color={C.acc}>+ Add</Btn>
              </div>
              <div style={{display:"flex",gap:4}}>
                <Btn onClick={()=>{setTSteps([]);resetAnim()}} color={C.pnk}>Clear All</Btn>
                <Btn onClick={()=>{setTSteps([{v:.5,w:0,dt:2},{v:.3,w:.5,dt:3}]);resetAnim()}} color={C.dim}>Reset Default</Btn>
              </div>
            </div>
            <div style={{...S.section,maxHeight:320,overflowY:"auto"}}>
              <div style={{color:C.dim,fontSize:11,fontFamily:"monospace",marginBottom:4}}>Steps ({tSteps.length})</div>
              {tSteps.length===0?<div style={{color:C.mut,fontSize:12,textAlign:"center",padding:8}}>No steps added yet.</div>:
              tSteps.map((s,i)=>{
                const wh=twistToWheel(s.v,s.w);
                return(
                <div key={i} style={{display:"flex",gap:6,alignItems:"center",fontSize:12,fontFamily:"monospace",padding:"3px 4px",borderBottom:`1px solid ${C.bd}`}}>
                  <span style={{color:C.dim,width:28}}>{i+1}</span>
                  <span style={{color:C.acc}}>v={s.v.toFixed(2)}</span>
                  <span style={{color:C.yel}}>ω={s.w.toFixed(2)}</span>
                  <span style={{color:C.blu}}>dt={s.dt.toFixed(1)}</span>
                  <span style={{color:C.mut,fontSize:10}}>→ φR={wh.pR.toFixed(1)} φL={wh.pL.toFixed(1)}</span>
                  <button onClick={()=>{setTSteps(p=>p.filter((_,j)=>j!==i));resetAnim()}} style={{background:"none",border:"none",color:C.mut,cursor:"pointer",fontSize:12}}>×</button>
                </div>
              )})}
            </div>
          </>)}

          {/* Interactive keyboard mode */}
          {mode==="interactive"&&(
            <div style={{display:"flex",flexDirection:"column",gap:10}}>
              <div style={S.section}>
                <div style={{color:C.yel,fontWeight:700,fontSize:14,fontFamily:"monospace",marginBottom:8}}>⌨ Keyboard Drive Mode</div>
                <div style={{color:C.dim,fontSize:12,fontFamily:"monospace",lineHeight:1.7,marginBottom:10}}>
                  Drive the robot in real-time using arrow keys. Each keypress generates (v, ω) commands that accumulate as SE(2) pose composition steps.
                </div>

                {/* Start / Stop button */}
                <div style={{display:"flex",justifyContent:"center",marginBottom:12}}>
                  <Btn onClick={()=>setIDriving(d=>!d)} color={iDriving?C.pnk:C.acc}
                    style={{fontSize:14,padding:"10px 28px",fontWeight:800,letterSpacing:1,
                      boxShadow:iDriving?`0 0 20px ${C.pnk}44`:`0 0 20px ${C.acc}44`}}>
                    {iDriving?"⏹ Stop Driving":"▶ Start Driving"}
                  </Btn>
                </div>
                {!iDriving&&iSteps.length===0&&(
                  <div style={{textAlign:"center",color:C.mut,fontSize:12,fontFamily:"monospace",marginBottom:8}}>
                    Click "Start Driving" then use arrow keys to move the robot.
                  </div>
                )}
                {iDriving&&(
                  <div style={{textAlign:"center",color:C.acc,fontSize:12,fontFamily:"monospace",marginBottom:8,
                    animation:"pulse 1.5s ease-in-out infinite",}}>
                    ● DRIVING — press arrow keys now
                  </div>
                )}

                {/* Key diagram */}
                <div style={{display:"flex",justifyContent:"center",marginBottom:12}}>
                  <div style={{display:"flex",flexDirection:"column",alignItems:"center",gap:4}}>
                    <div style={{
                      width:48,height:40,borderRadius:8,display:"flex",alignItems:"center",justifyContent:"center",
                      fontFamily:"monospace",fontSize:14,fontWeight:700,
                      background:liveV>0?C.acc+"44":C.mBg,color:liveV>0?C.acc:C.dim,
                      border:`2px solid ${liveV>0?C.acc:C.mBd}`,transition:"all 0.1s",
                    }}>↑</div>
                    <div style={{display:"flex",gap:4}}>
                      <div style={{
                        width:48,height:40,borderRadius:8,display:"flex",alignItems:"center",justifyContent:"center",
                        fontFamily:"monospace",fontSize:14,fontWeight:700,
                        background:liveW>0?C.yel+"44":C.mBg,color:liveW>0?C.yel:C.dim,
                        border:`2px solid ${liveW>0?C.yel:C.mBd}`,transition:"all 0.1s",
                      }}>←</div>
                      <div style={{
                        width:48,height:40,borderRadius:8,display:"flex",alignItems:"center",justifyContent:"center",
                        fontFamily:"monospace",fontSize:14,fontWeight:700,
                        background:liveV<0?C.pnk+"44":C.mBg,color:liveV<0?C.pnk:C.dim,
                        border:`2px solid ${liveV<0?C.pnk:C.mBd}`,transition:"all 0.1s",
                      }}>↓</div>
                      <div style={{
                        width:48,height:40,borderRadius:8,display:"flex",alignItems:"center",justifyContent:"center",
                        fontFamily:"monospace",fontSize:14,fontWeight:700,
                        background:liveW<0?C.yel+"44":C.mBg,color:liveW<0?C.yel:C.dim,
                        border:`2px solid ${liveW<0?C.yel:C.mBd}`,transition:"all 0.1s",
                      }}>→</div>
                    </div>
                  </div>
                </div>

                {/* Mapping legend */}
                <div style={{fontSize:12,fontFamily:"monospace",color:C.dim,lineHeight:1.8}}>
                  <div><span style={{color:C.acc}}>↑ / ↓</span> = forward / backward <span style={{color:C.txt}}>(v = ±{V_SPEED} m/s)</span></div>
                  <div><span style={{color:C.yel}}>← / →</span> = turn left / right <span style={{color:C.txt}}>(ω = ±{W_SPEED} rad/s)</span></div>
                  <div style={{marginTop:4,color:C.txt}}>Hold multiple keys for combined motion (arc trajectories).</div>
                </div>
              </div>

              {/* Live state */}
              <div style={{...S.section,background:C.mBg,border:`1px solid ${C.mBd}`}}>
                <div style={{color:C.dim,fontSize:11,fontFamily:"monospace",marginBottom:6}}>Live Command</div>
                <div style={{display:"flex",gap:16,fontSize:14,fontFamily:"monospace",fontWeight:700}}>
                  <span style={{color:liveV!==0?C.acc:C.mut}}>v = {liveV.toFixed(1)} m/s</span>
                  <span style={{color:liveW!==0?C.yel:C.mut}}>ω = {liveW.toFixed(1)} rad/s</span>
                </div>
                <div style={{marginTop:8,display:"flex",gap:12,fontSize:12,fontFamily:"monospace",color:C.dim}}>
                  <span>Steps recorded: <span style={{color:C.txt}}>{iSteps.length}</span></span>
                  <span>Time: <span style={{color:C.txt}}>{(iSteps.length*LIVE_DT).toFixed(1)}s</span></span>
                </div>
              </div>

              {/* Clear / Undo */}
              <div style={{display:"flex",gap:6}}>
                <Btn onClick={()=>{setISteps([]);resetAnim()}} color={C.pnk}>Clear Path</Btn>
                <Btn onClick={()=>{setISteps(p=>p.slice(0,Math.max(0,p.length-20)));resetAnim()}} color={C.dim} disabled={iSteps.length===0}>Undo (20 steps)</Btn>
              </div>
            </div>
          )}

          {/* Preset info */}
          {mode==="preset"&&(
            <div style={S.section}>
              <div style={{color:C.acc,fontWeight:700,fontSize:14,fontFamily:"monospace",marginBottom:8}}>Letter Tracing Presets</div>
              <div style={{color:C.dim,fontSize:12,fontFamily:"monospace",lineHeight:1.6}}>
                <p style={{margin:"0 0 8px"}}>Select SLAM, DGIST, or ROBOT above. Each preset contains hundreds of differential drive commands that trace the corresponding letters.</p>
                <p style={{margin:"0 0 8px"}}>These presets were generated with r=0.1, l=0.2. Try changing r and l to see how the trajectory deforms!</p>
                <p style={{margin:0}}>Press <span style={{color:C.acc}}>▶ Animate</span> to watch the robot draw each letter step by step. Use the slider to scrub through the animation.</p>
              </div>
              {preset&&(
                <div style={{marginTop:8,padding:"6px 8px",background:C.mBg,borderRadius:6,fontSize:12,fontFamily:"monospace",color:C.dim}}>
                  <span style={{color:C.acc}}>{preset}</span>: {presetSteps.length} steps, {totalTime.toFixed(1)}s total
                </div>
              )}
            </div>
          )}

          {/* Reference */}
          <div style={{...S.section,background:C.mBg,border:`1px solid ${C.mBd}`}}>
            <div style={{color:C.acc,fontWeight:700,fontSize:13,fontFamily:"monospace",marginBottom:6}}>Quick Reference</div>
            <div style={{fontSize:12,fontFamily:"monospace",color:C.dim,lineHeight:1.6}}>
              <div><span style={{color:C.blu}}>v = r(φR + φL) / 2</span> — linear velocity</div>
              <div><span style={{color:C.blu}}>ω = r(φR - φL) / (2l)</span> — angular velocity</div>
              <div style={{marginTop:6}}>
                <span style={{color:C.yel}}>Nonholonomic:</span> ẏ_local = 0 (no sideways slip)
              </div>
              <div style={{marginTop:4}}>
                φR = φL → straight | φR = -φL → spin in place | one = 0 → pivot
              </div>
              <div style={{marginTop:6,color:C.txt}}>
                Composition: <sup>W</sup><sub>n</sub>T = <sup>0</sup><sub>1</sub>T · <sup>1</sup><sub>2</sub>T ··· <sup>n-1</sup><sub>n</sub>T
              </div>
            </div>
          </div>
        </div>
      </div>

      <div style={{textAlign:"center",marginTop:16,color:C.mut,fontSize:11,fontFamily:"monospace"}}>
        DGIST · Dept. of Robotics & Mechatronics · RT604 SLAM Course
      </div>
    </div>
  );
}
