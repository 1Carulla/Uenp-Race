/*
  Seguidor de linha por camera + comandos pela rede. AI Thinker ESP32-CAM.
  GPIO13 desativado; GPIO14 ENA/esquerda; GPIO15 ENB/direita; IN1/IN3=3V3; IN2/IN4=GND.
  O navegador analisa a imagem. Mantenha a pagina aberta e visivel ao seguir a linha.
  HTML e JavaScript incorporados: basta gravar este .ino.
*/
// ===== AJUSTES DO MOVIMENTO: altere aqui e grave novamente =====
#define MOTOR_PWM 220         // Potencia preservada do arquivo local da 10.5.
#define PASSO_RETA_MS 150      // Tempo andando em linha reta, em milissegundos.
#define PASSO_CURVA_MS 150    // Tempo de uma curva moderada.
#define PASSO_FECHADA_MS 150   // Curva forte ou leitura parcial da linha.
#define PAUSA_OBSERVACAO_MS 30 // Pausa entre pulsos; exige nova imagem apos a pausa.
#define CONFIRMA_PERDA_MS 300  // Confirma perda persistente; nao cria pulsos sem linha.
#define LIMITE_SEM_LINHA_MS 3000 // Prazo total para reencontrar a linha sem novo clique.
// Tempos de movimento permitidos: 60 a 260 ms. 1000 ms = 1 segundo.
// A pausa deve ficar entre 20 e 500 ms. Nao altere os watchdogs para regular velocidade.
// ===== FIM DOS AJUSTES =====

#define MOTION_TEXT_INNER(x) #x
#define MOTION_TEXT(x) MOTION_TEXT_INNER(x)
static_assert(MOTOR_PWM >= 0 && MOTOR_PWM <= 255, "MOTOR_PWM: use 0 a 255");
static_assert(PASSO_RETA_MS >= 60 && PASSO_RETA_MS <= 260, "PASSO_RETA_MS: use 60 a 260");
static_assert(PASSO_CURVA_MS >= 60 && PASSO_CURVA_MS <= 260, "PASSO_CURVA_MS: use 60 a 260");
static_assert(PASSO_FECHADA_MS >= 60 && PASSO_FECHADA_MS <= 260, "PASSO_FECHADA_MS: use 60 a 260");
static_assert(PAUSA_OBSERVACAO_MS >= 20 && PAUSA_OBSERVACAO_MS <= 500, "PAUSA_OBSERVACAO_MS: use 20 a 500");
static_assert(CONFIRMA_PERDA_MS >= 100 && CONFIRMA_PERDA_MS <= 600, "CONFIRMA_PERDA_MS: use 100 a 600");
static_assert(LIMITE_SEM_LINHA_MS > CONFIRMA_PERDA_MS && LIMITE_SEM_LINHA_MS <= 3000, "LIMITE_SEM_LINHA_MS: maior que confirmacao, maximo 3000");

#include <Arduino.h>
#include <WiFi.h>
#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_wifi.h"
#include "esp_core_dump.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "cJSON.h"
#include "lwip/sockets.h"
#include "lwip/tcp.h"
#include <math.h>
#include <new>
#include <errno.h>
#include <atomic>

// ===== REDE PROPRIA =====
const char *AP_SSID="Carrinho-10.4";
const char *AP_PASSWORD="Carrinho104";
#define AP_CHANNEL 0 // 0: escolher 1/6/11 no boot; ou fixe 1, 6 ou 11.
#define AP_TX_POWER WIFI_POWER_15dBm
// 1: JPEG continuo com dois buffers na PSRAM. 0: captura da 10.5.
#define CAMERA_BUFFER_DUPLO 1
static_assert(AP_CHANNEL==0||AP_CHANNEL==1||AP_CHANNEL==6||AP_CHANNEL==11,"Canal: 0, 1, 6 ou 11");
constexpr int SERVO_PIN=13, ENA_PIN=14, ENB_PIN=15;
constexpr ledc_mode_t PWM_MODE=LEDC_LOW_SPEED_MODE;
// Camera: timer0/canal0. Motores: timer1/canais2,3. Sem PWM de servo.
constexpr ledc_channel_t LEFT_CH=LEDC_CHANNEL_2, RIGHT_CH=LEDC_CHANNEL_3;

// BEGIN CONTROL_CORE
class DriveControl {
public:
  enum Mode { IDLE, AUTO_LINE };
  enum Reason { BOOT, USER_STOP, NO_COMMAND, WIFI_LOST, LINE_LOST, FAULT };
  static constexpr uint32_t COMMAND_TIMEOUT_MS = 300;
  static constexpr uint32_t FRAME_MAX_AGE_MS = 300;
  static constexpr uint32_t OBSERVE_MS = PAUSA_OBSERVACAO_MS;
  static constexpr uint32_t MAX_STEP_MS = 260;
  static constexpr uint32_t LOSS_CONFIRM_MS = CONFIRMA_PERDA_MS;
  static constexpr uint32_t LOSS_TIMEOUT_MS = LIMITE_SEM_LINHA_MS;
  static constexpr int MAX_PWM = MOTOR_PWM;
  bool armed = false;
  int targetLeft = 0, targetRight = 0;
  float outputLeft = 0, outputRight = 0;
  uint32_t token = 0, owner = 0, lastSeq = 0;
  Mode mode = IDLE;
  Reason reason = BOOT;

  bool arm(uint32_t now, uint32_t newToken, uint32_t session, Mode requested) {
    if (armed || !newToken || !session || requested != AUTO_LINE) return false;
    armed = true; token = newToken; owner = session; mode = requested;
    lastCommand = stoppedAt = now; lastSeq = lastFrame = 0;
    moving = false; lossActive = ambiguityHold = false; recoverFrames = 0; targetLeft = targetRight = 0; outputLeft = outputRight = 0;
    return true;
  }
  bool command(uint32_t now, uint32_t receivedToken, uint32_t session,
               uint32_t sequence, int left, int right, uint32_t duration,
               uint32_t frameId, uint32_t frameAge, uint32_t line) {
    tick(now, true);
    if (!armed || token != receivedToken || owner != session) return false;
    if (line > 2 || !sequence || sequence <= lastSeq || left < 0 || right < 0 ||
        left > MAX_PWM || right > MAX_PWM || duration < 60 || duration > MAX_STEP_MS) return false;
    if (!frameId || frameId <= lastFrame || frameAge > FRAME_MAX_AGE_MS) {
      stop(LINE_LOST); return false;
    }
    lastSeq = sequence; lastFrame = frameId; lastCommand = now;
    if (line != 1) {
      if (!lossActive) { lossActive = true; lossAt = now; }
      recoverFrames = 0;
      if (line == 2) { ambiguityHold = true; finishStep(now); }
      // A missed frame may finish the original pulse, never start or extend one.
      return true;
    }
    if (lossActive) {
      const bool confirmed = ambiguityHold || static_cast<uint32_t>(now-lossAt) >= LOSS_CONFIRM_MS;
      if (!confirmed || ++recoverFrames >= 2) { lossActive = ambiguityHold = false; recoverFrames = 0; }
      else { finishStep(now); return true; }
    }
    if (left == 0 && right == 0) { finishStep(now); return true; }
    // Frames are a heartbeat during movement, never a queue or an extension.
    if (moving) return true;
    const uint32_t stationary = static_cast<uint32_t>(now - stoppedAt);
    if (stationary < OBSERVE_MS || frameAge > stationary - OBSERVE_MS) return true;
    moving = true; stepStarted = now; stepDuration = duration;
    targetLeft = left; targetRight = right;
    outputLeft = outputRight = 0;
    return true;
  }
  void stop(Reason why) {
    armed = false; moving = false; mode = IDLE; reason = why; token = owner = 0;
    targetLeft = targetRight = 0; outputLeft = outputRight = 0;
  }
  void expire(uint32_t now, bool connected) {
    if (!armed) return;
    if (!connected) stop(WIFI_LOST);
    else if (static_cast<uint32_t>(now - lastCommand) >= COMMAND_TIMEOUT_MS) stop(NO_COMMAND);
    else if (lossActive && static_cast<uint32_t>(now-lossAt) >= LOSS_TIMEOUT_MS) stop(LINE_LOST);
    if (armed && lossActive && static_cast<uint32_t>(now-lossAt) >= LOSS_CONFIRM_MS) finishStep(now);
  }
  void tick(uint32_t now, bool connected) {
    expire(now, connected);
    if (!armed || !moving) return;
    const uint32_t elapsed = static_cast<uint32_t>(now - stepStarted);
    if (elapsed >= stepDuration) { finishStep(now); return; }
    // Short ramp; no blocking delay. Timing is enforced on the ESP32 itself.
    const float fraction = elapsed >= 40 ? 1.0f : elapsed / 40.0f;
    outputLeft = targetLeft * fraction; outputRight = targetRight * fraction;
  }
  uint32_t commandAge(uint32_t now) const { return static_cast<uint32_t>(now - lastCommand); }
  uint32_t remaining(uint32_t now) const {
    const uint32_t elapsed = static_cast<uint32_t>(now - stepStarted);
    return armed && moving && elapsed < stepDuration ? stepDuration - elapsed : 0;
  }
  const char *phaseText() const { return !armed ? "parado" : moving ? "movendo" : "observando"; }
  const char *reasonText() const {
    if (armed) return moving ? "passo curto pelos motores" : "parado para observar";
    switch (reason) {
      case USER_STOP: return "parada solicitada";
      case NO_COMMAND: return "300 ms sem comando valido";
      case WIFI_LOST: return "Wi-Fi desconectado";
      case LINE_LOST: return "imagem antiga ou linha nao confirmada no prazo";
      case FAULT: return "falha nos atuadores";
      default: return "aguardando comando";
    }
  }
private:
  bool moving = false, lossActive = false, ambiguityHold = false;
  uint32_t lossAt = 0, recoverFrames = 0;
  uint32_t lastCommand=0, lastFrame=0, stoppedAt=0, stepStarted=0, stepDuration=0;
  void finishStep(uint32_t now) {
    if (moving) stoppedAt = now;
    moving = false; targetLeft = targetRight = 0; outputLeft = outputRight = 0;
  }
};
// END CONTROL_CORE

// Call metadata methods under stateMux. Copy/send bytes outside the lock.
class FrameCache {
public:
  struct Slot {uint8_t *data=nullptr;size_t len=0;uint32_t captured=0,id=0;bool reading=false,writing=false;};
  Slot slots[2];int newest=-1;
  int reserveWrite(){
    // Prefer the spare slot, keeping the latest JPEG available while copying.
    for(int i=0;i<2;i++)if(i!=newest&&!slots[i].reading&&!slots[i].writing){slots[i].writing=true;return i;}
    // A slow sender may hold the older slot. Refresh an unread latest frame
    // instead of freezing it until the network releases the older frame.
    // acquire() rejects writing slots, so no reader sees a partial JPEG.
    if(newest>=0&&!slots[newest].reading&&!slots[newest].writing){slots[newest].writing=true;return newest;}
    return -1;
  }
  void publish(int i,size_t len,uint32_t captured,uint32_t id){slots[i].len=len;slots[i].captured=captured;slots[i].id=id;slots[i].writing=false;newest=i;}
  int acquire(uint32_t now,uint32_t seen){
    if(newest<0)return -1;
    auto &s=slots[newest];
    if(s.writing||s.reading||!s.id||s.id==seen||static_cast<uint32_t>(now-s.captured)>200)return -1;
    s.reading=true;return newest;
  }
  void release(int i){slots[i].reading=false;}
  void invalidate(){newest=-1;}
};


DriveControl drive;
portMUX_TYPE stateMux=portMUX_INITIALIZER_UNLOCKED;
bool actuatorsReady=false;
std::atomic<bool> cameraReady{false};
std::atomic<bool> apReady{false},apHasClient{false};
std::atomic<uint32_t> apLastReason{0},captureFailures{0},captureDrops{0},cameraLastBytes{0},controlClosed{0};
std::atomic<uint32_t> captureLastMs{0},captureAgeAtReadMs{0},cameraAgeAtSendMs{0},cameraDriverBuffers{0};
int selectedChannel=6;int32_t channelScores[3]={0,0,0};
std::atomic<int> apConfigError{0},apActualPower{-1};
std::atomic<uint32_t> cameraSent{0},cameraWaits{0},cameraSendErrors{0},controlSendErrors{0};
std::atomic<uint32_t> cameraSendLastMs{0},cameraSendMaxMs{0},controlSendLastMs{0},wifiDisconnects{0};
std::atomic<esp_err_t> cameraError{ESP_OK};
esp_err_t actuatorError=ESP_OK;
httpd_handle_t webServer=nullptr,frameServer=nullptr;
struct FrameStamp { uint32_t id=0, captured=0; };
FrameStamp frameStamps[8];
uint32_t nextFrame=0;
uint32_t bootId=0;
char savedCrash[1024]="Nenhum registro de travamento disponivel.";
struct ClientSession { uint32_t id; };
FrameCache cameraCache;
size_t cameraCacheCapacity=0;
constexpr size_t CAMERA_PREFIX=30; // 10 bytes RFC6455 + 20 bytes image metadata.
std::atomic<uint32_t> cameraDemandAt{0};

const char PAGE_HTML[] PROGMEM=R"LINEPAGE(
<!doctype html>
<html lang="pt-BR"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Carrinho · seguir linha</title>
<style>
:root{font-family:system-ui,sans-serif;color-scheme:dark;color:#eaf4f6;background:#101b24}*{box-sizing:border-box}body{max-width:880px;margin:auto;padding:24px}h1{margin-bottom:6px;font-size:32px}h2{font-size:19px;margin:0 0 16px}p,small{color:#b6c8d3;line-height:1.5}section{background:#192b38;border:1px solid #365063;border-radius:14px;padding:20px;margin-top:18px}.actions{display:flex;gap:10px;flex-wrap:wrap}button{font:inherit;font-weight:650;border:0;padding:12px 16px;border-radius:8px;background:#78ddbe;color:#102820;cursor:pointer}button.danger{background:#f08383;color:#350b0b}button:disabled{opacity:.4;cursor:not-allowed}canvas{width:100%;aspect-ratio:4/3;background:#0b1219;border-radius:9px}.metrics{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin:14px 0}.metrics div{background:#10212c;border-radius:8px;padding:10px;font-size:13px}.metrics strong{display:block;font-size:19px;color:#78ddbe}#notice{font-weight:600;line-height:1.5;white-space:pre-line}#device{white-space:pre-line;font-size:13px;color:#b6c8d3;line-height:1.6}#log{max-height:170px;overflow:auto;font-size:13px;line-height:1.7;padding-left:20px}button:focus-visible{outline:3px solid white;outline-offset:3px}@media(max-width:760px){body{padding:14px}.metrics{grid-template-columns:1fr 1fr 1fr}h1{font-size:27px}}
</style></head><body>
<h1>Seguir a linha preta</h1><p><strong>V10.6 · Bifurcações estáveis · Rede direta</strong></p><p>O carrinho observa a linha, anda um pouco e para para olhar novamente. A diferença entre os motores faz as curvas. Nas bifurcações, escolhe a continuação mais reta da faixa.</p>
<section><div class="actions"><button id="start" disabled>Iniciar seguidor</button><button id="stop" class="danger">PARAR</button></div>
<p id="notice" role="status">Conectando ao carrinho…</p><small>Se a linha sumir por um instante, conclui apenas o pulso atual e aguarda reencontrá-la por até 3 segundos. Nas bifurcações, prioriza a continuação mais reta visível. Sem imagem ou comunicação, encerra o ciclo. Mantenha esta página aberta e visível.</small></section>
<section><h2>Câmera e linha detectada</h2><canvas id="view" width="640" height="480" aria-label="Imagem da câmera com marcação da linha"></canvas>
<div class="metrics"><div><strong id="fps">—</strong>imagens/s</div><div><strong id="age">—</strong>idade estimada da imagem</div><div><strong id="rtt">—</strong>ida e volta do comando</div></div>
<p id="cameraNetwork">Câmera: conectando ao canal de imagens…</p>
<p id="visionState" role="status">Aguardando imagem</p><p id="pathState" role="status">Caminho: aguardando a linha</p><div id="device"></div></section>

<section><p><a href="/rede" target="_blank" rel="noopener">Diagnóstico da rede e câmera atuais</a></p><h2>Eventos</h2><ol id="log" aria-live="polite"></ol><p><a href="/diagnostico" target="_blank" rel="noopener">Ver registro da última falha salva na ESP32</a> — pode pertencer a uma inicialização anterior.</p></section>
<script>
globalThis.CARRINHO_MOTION={pwm:)LINEPAGE" MOTION_TEXT(MOTOR_PWM) R"LINEPAGE(,straightMs:)LINEPAGE" MOTION_TEXT(PASSO_RETA_MS) R"LINEPAGE(,curveMs:)LINEPAGE" MOTION_TEXT(PASSO_CURVA_MS) R"LINEPAGE(,sharpMs:)LINEPAGE" MOTION_TEXT(PASSO_FECHADA_MS) R"LINEPAGE(,pauseMs:)LINEPAGE" MOTION_TEXT(PAUSA_OBSERVACAO_MS) R"LINEPAGE(,confirmMs:)LINEPAGE" MOTION_TEXT(CONFIRMA_PERDA_MS) R"LINEPAGE(,lossMs:)LINEPAGE" MOTION_TEXT(LIMITE_SEM_LINHA_MS) R"LINEPAGE(};
</script>
<script>
/* Analise da linha no navegador. Sem dependencias externas. */
class LineVision {
  constructor(){this.previousPath=null;this.previousAnchor=null;this.previousX=null;this.gray=null;this.hist=new Uint32Array(256);this.profile=null;this.depth=null;}
  reset(){this.previousPath=null;this.previousAnchor=null;this.previousX=null;}
  detect(rgba,w,h,cfg={}){
    const top=Math.round(h*Math.max(0.15,Math.min(0.65,cfg.roiTop??0.35)));
    const bottom=Math.min(h-3,Math.round(h*Math.max(0.75,Math.min(0.95,cfg.roiBottom??0.90))));
    if(!this.gray||this.gray.length!==w*h)this.gray=new Uint8Array(w*h);
    if(!this.profile||this.profile.length!==w){this.profile=new Float32Array(w);this.depth=new Float32Array(w);}
    const gray=this.gray,hist=this.hist;hist.fill(0);
    let count=0,sum=0;
    for(let y=Math.max(0,top-2);y<=bottom+2;y++)for(let x=0;x<w;x++){
      const p=(y*w+x)*4,g=(77*rgba[p]+150*rgba[p+1]+29*rgba[p+2])>>8;
      gray[y*w+x]=g;
      if(y>=top&&y<=bottom&&x>=2&&x<w-2){hist[g]++;count++;sum+=g;}
    }
    let leftN=0,leftSum=0,best=-1,darkMean=0,lightMean=0;
    for(let t=0;t<255;t++){
      leftN+=hist[t];leftSum+=t*hist[t];const rightN=count-leftN;
      if(!leftN||!rightN)continue;
      const a=leftSum/leftN,b=(sum-leftSum)/rightN;
      const score=leftN*rightN*(a-b)*(a-b);
      if(score>best){best=score;darkMean=a;lightMean=b;}
    }
    const contrast=lightMean-darkMean;
    const threshold=cfg.autoThreshold===false?Number(cfg.threshold??100):Math.round((darkMean+lightMean)/2);
    const fail=(reason,points=[],ambiguous=false)=>{this.previousPath=null;this.previousAnchor=null;return {valid:false,reason,points,threshold,contrast,top,bottom,confidence:0,ambiguous};};
    if(best<0)return fail('Pouco contraste entre a linha e o piso');
    // A shadow can occupy most of the image. Validate local tape edges instead
    // of rejecting the whole frame by the global percentage of dark pixels.
    // Keep the original seven near rows, then inspect two additional rows.
    const nearTop=Math.max(top,Math.round(h*0.50));
    const rows=Array.from({length:7},(_,i)=>Math.round(bottom-(bottom-nearTop)*i/6));
    if(top<nearTop)rows.push(Math.round((nearTop+top)/2),top);
    const points=[],crossRows=[],anchor=this.previousAnchor;
    let expected=this.previousX??(w-1)/2,previousWidth=anchor?.width??null;
    let routeSlope=anchor?.slope??0,branch=false;
    for(let row=0;row<rows.length;row++){
      const y=rows[row],runs=[];
      // Project the incoming direction, not the centre of a widened junction.
      if(points.length){const last=points.at(-1);expected=last.x+routeSlope*(last.y-y);}
      else if(anchor)expected=anchor.x+anchor.slope*(anchor.y-y);
      let broad=false;
      // At 160 px: >=6 px near the car, >=4 px farther away. Reject thin
      // grout even when it is closer to the centre than the actual tape.
      const minWidth=Math.max(3,Math.round(w*(0.035-0.010*Math.min(row,6)/6)));
      const profile=this.profile,depth=this.depth,radius=Math.ceil(w*0.22);
      for(let x=0;x<w;x++)profile[x]=(gray[(y-1)*w+x]+gray[y*w+x]+gray[(y+1)*w+x])/3;
      // A tape valley must have brighter floor on BOTH sides. A monotonic
      // lighting gradient/shadow edge alone has no such valley.
      for(let x=0;x<w;x++){
        let left=profile[x],right=profile[x];
        for(let k=Math.max(0,x-radius);k<x;k++)left=Math.max(left,profile[k]);
        for(let k=x+1;k<=Math.min(w-1,x+radius);k++)right=Math.max(right,profile[k]);
        depth[x]=Math.min(left,right)-profile[x];
      }
      let begin=-1;
      for(let x=1;x<w-1;x++){
        const dark=depth[x]>=24&&(cfg.autoThreshold!==false||profile[x]<=threshold);
        if(dark&&begin<0)begin=x;
        if(begin>=0&&(!dark||x===w-2)){
          let end=dark?x:x-1,localContrast=0;
          for(let k=begin;k<=end;k++)localContrast=Math.max(localContrast,depth[k]);
          // Trim blurred shoulders relative to the valley depth: a thin grout
          // line must not become thick just because its edges were averaged.
          const edge=Math.max(24,localContrast*.45);
          while(begin<=end&&depth[begin]<edge)begin++;
          while(end>=begin&&depth[end]<edge)end--;
          const width=end-begin+1;
          if(width>w*0.40&&begin<=expected&&end>=expected)broad=true;
          if(width>=minWidth&&width<=w*0.40&&begin>1&&end<w-2){
            const center=(begin+end)/2;
            runs.push({x:center,y,width,begin,end,localContrast,score:Math.abs(center-expected)+(previousWidth?Math.abs(width-previousWidth)*0.3:0)});
          }
          begin=-1;
        }
      }
      if(!runs.length){
        // Preserve the stop on a broad transverse dark strip/crossing.
        let start=-1;
        for(let x=1;x<w-1;x++){
          const dark=profile[x]<=threshold;
          if(dark&&start<0)start=x;
          if(start>=0&&(!dark||x===w-2)){
            const end=dark?x:x-1;
            if(end-start+1>w*.40&&start<=expected&&end>=expected)broad=true;
            start=-1;
          }
        }
        if(broad){branch=true;crossRows.push(y);}
        continue;
      }
      // A merged branch has no reliable centre. Skip it, requiring measured
      // tape on both sides of the junction instead of inventing a midpoint.
      const narrow=previousWidth?runs.filter(p=>p.width<=previousWidth*1.55+1):runs;
      if(!narrow.length){branch=true;crossRows.push(y);continue;}
      if(narrow.length<runs.length)branch=true;
      // After anchoring, do not jump to a wide object elsewhere in the image.
      const nearby=points.length?narrow.filter(p=>Math.abs(p.x-expected)<=w*0.2):narrow;
      if(!nearby.length)break;
      const widest=Math.max(...nearby.map(p=>p.width));
      const candidates=nearby.filter(p=>p.width>=widest*0.65);
      candidates.sort((a,b)=>a.score-b.score);
      if(runs.length>1)branch=true;
      let chosen=candidates[0];
      const prior=this.previousPath?.find(p=>p.y===y);
      if(prior&&candidates.length>1){
        const close=candidates.filter(p=>p.score<=candidates[0].score+w*0.06);
        // Only compare real, sufficiently thick candidates in the current image.
        close.sort((a,b)=>Math.abs(a.x-prior.x)-Math.abs(b.x-prior.x));
        chosen=close[0];
      }
      if(points.length){
        const last=points.at(-1),slope=(chosen.x-last.x)/(last.y-y);
        // Preserve the incoming heading while choosing between branches.
        if(!branch)routeSlope=points.length===1?slope:0.6*routeSlope+0.4*slope;
      }
      points.push(chosen);expected=chosen.x;previousWidth=chosen.width;
    }
    // Three coherent nearby samples suffice. Avoid inferring a route from
    // isolated far-away fragments or fitting a sharp curve to just 3 points.
    const rowStep=(bottom-nearTop)/6;
    const nearGap=points.length?bottom-points[0].y:Infinity;
    const nearSkipped=points.length?rows.filter(y=>y>points[0].y):[];
    const crossingNear=anchor&&nearSkipped.length>0&&nearSkipped.length<=2&&nearSkipped.every(y=>crossRows.includes(y));
    if(points.length<3||(nearGap>rowStep+1&&!crossingNear))return fail('Trajeto insuficiente: precisa de 3 pontos próximos',points);
    for(let i=1;i<points.length;i++){
      const gap=points[i-1].y-points[i].y;
      if(gap>rowStep*2+1){
        const skipped=rows.filter(y=>y<points[i-1].y&&y>points[i].y);
        if(skipped.length>2||!skipped.every(y=>crossRows.includes(y)))return fail('Linha interrompida na area de leitura',points);
      }
    }
    // A large dark patch without an exit is not evidence of a traversable fork.
    if(crossRows.length&&crossRows.some(y=>(y>=nearTop&&y<points.at(-1).y)||(y>points[0].y&&!crossingNear)))
      return fail('Aguardando faixa visivel depois da juncao',points);
    // At least three original near rows must still identify actual tape.
    if(points.filter(p=>p.y>=nearTop).length<3)return fail('Trajeto insuficiente: precisa de 3 pontos próximos',points);
    const limitedNear=nearGap>3;
    const partial=points.length<5;
    const near=(points[0].x+points[1].x)/2,far=(points.at(-1).x+points.at(-2).x)/2;
    // Quadratic fit accepts smooth bends instead of requiring a straight line.
    // Coordinates are normalized image positions, not physical centimetres.
    const degree=partial?2:3;
    const matrix=Array.from({length:degree},()=>Array(degree+1).fill(0));
    for(const p of points){const t=(bottom-p.y)/(bottom-top),basis=[1,t,t*t];
      for(let i=0;i<degree;i++){for(let j=0;j<degree;j++)matrix[i][j]+=basis[i]*basis[j];matrix[i][degree]+=basis[i]*p.x;}}
    for(let i=0;i<degree;i++){
      let pivot=i;for(let j=i+1;j<degree;j++)if(Math.abs(matrix[j][i])>Math.abs(matrix[pivot][i]))pivot=j;
      [matrix[i],matrix[pivot]]=[matrix[pivot],matrix[i]];
      if(Math.abs(matrix[i][i])<1e-8)return fail('Trajeto insuficiente',points);
      const divisor=matrix[i][i];for(let k=i;k<=degree;k++)matrix[i][k]/=divisor;
      for(let j=0;j<degree;j++)if(j!==i){const factor=matrix[j][i];for(let k=i;k<=degree;k++)matrix[j][k]-=factor*matrix[i][k];}
    }
    const curve=matrix.map(row=>row[degree]),n=points.length;if(partial)curve.push(0);
    const residual=Math.sqrt(points.reduce((s,p)=>{const t=(bottom-p.y)/(bottom-top);return s+(p.x-(curve[0]+curve[1]*t+curve[2]*t*t))**2;},0)/n);
    if(residual>w*0.045)return fail('Trajeto irregular ou ambíguo',points);
    const localContrast=points.reduce((sum,p)=>sum+p.localContrast,0)/n;
    const confidence=Math.min(1,localContrast/60)*(0.65+0.35*Math.min(n,7)/7)*Math.max(0,1-residual/(w*0.12))*(limitedNear?0.85:1);
    if(confidence<0.48)return fail('Deteccao pouco confiavel',points);
    this.previousX=near;this.previousPath=points.map(p=>({x:p.x,y:p.y}));
    const widths=points.map(p=>p.width).sort((a,b)=>a-b),nominalWidth=widths[Math.floor(widths.length/2)];
    const stable=points.find(p=>p.width<=nominalWidth*1.2)||points[0];
    this.previousAnchor={x:stable.x,y:stable.y,slope:routeSlope,width:nominalWidth};
    const center=(w-1)/2,heading=(far-near)/center;
    const lookAhead=Math.max(0.2,Math.min(1.2,Number(cfg.lookAhead??0.8)));
    const target=near+lookAhead*(far-near);
    const error=Math.max(-1,Math.min(1,(target-center)/center));
    const direction=heading>0.12?'Curva à direita':heading< -0.12?'Curva à esquerda':'Reta';
    return {valid:true,reason:branch?'Bifurcacao: continuacao mais reta':limitedNear||partial?'Linha encontrada; leitura parcial':'Linha encontrada',limitedNear,partial,speedScale:limitedNear||partial?0.65:1,direction,heading,target,curve,points,near,far,error,confidence,threshold,contrast,localContrast,top,bottom,branch};
  }
}
if(typeof module!=='undefined'&&module.exports)module.exports={LineVision};

</script>
<script>
// One observation per new image; timers never invent observations.
class LineRecovery {
  constructor(settings=globalThis.CARRINHO_MOTION){this.settings=settings;this.reset();}
  reset(){this.lostAt=null;this.recoverFrames=0;this.ambiguityHold=false;this.ended=false;}
  confirmed(now){return this.lostAt!==null&&(this.ambiguityHold||now-this.lostAt>=this.settings.confirmMs);}
  observe(line,now){
    if(this.expired(now)){this.ended=true;return;}
    const valid=!!line?.valid&&!line.ambiguous&&Number.isFinite(line.error);
    if(!valid){
      if(this.lostAt===null)this.lostAt=now;
      this.recoverFrames=0;if(line?.ambiguous)this.ambiguityHold=true;
    }else if(this.lostAt!==null){
      // A transient miss does not cost two more frames. After a confirmed
      // loss or ambiguity, require two valid images before another pulse.
      if(!this.confirmed(now)||++this.recoverFrames>=2){
        this.lostAt=null;this.recoverFrames=0;this.ambiguityHold=false;
      }
    }
  }
  expired(now){return this.ended||(this.lostAt!==null&&now-this.lostAt>=this.settings.lossMs);}
}
if(typeof module!=='undefined'&&module.exports)module.exports={LineRecovery};

</script>
<script>
// ENA: left wheel; ENB: right wheel, viewed from behind. No reverse required.
function differentialStep(line, recovering=false, swapMotors=false) {
  if (!line?.valid || line.ambiguous || recovering || !Number.isFinite(line.error))
    return {left:0,right:0,duration:100,direction:'aguardando a linha'};
  const error=Math.max(-1,Math.min(1,line.error)), amount=Math.abs(error);
  const settings=globalThis.CARRINHO_MOTION;
  const outer=settings.pwm; // Shared with firmware, editable at the top of .ino.
  const inner=amount<0.10?outer:amount>=0.40?0:Math.round(outer*(1-(amount-0.10)/0.30));
  let left=error>=0?outer:inner,right=error>=0?inner:outer;
  if(swapMotors)[left,right]=[right,left];
  const duration=line.partial||line.limitedNear?settings.sharpMs:amount<0.10?settings.straightMs:amount<0.40?settings.curveMs:settings.sharpMs;
  return {left,right,duration,direction:amount<0.10?'reto':error>0?'direita':'esquerda'};
}
if(typeof module!=='undefined'&&module.exports)module.exports={differentialStep};

</script>
<script>
/* One outstanding image request; no Base64, HTTP polling or image backlog. */
function parseCameraFrame(buffer){
  if(!(buffer instanceof ArrayBuffer)||buffer.byteLength<24)throw Error('Imagem incompleta');
  const v=new DataView(buffer);
  if(v.getUint32(0,true)!==0x374d4143)throw Error('Versao da camera diferente. Recarregue a pagina.');
  const id=v.getUint32(4,true),age=v.getUint32(8,true),service=v.getUint32(12,true),size=v.getUint32(16,true);
  if(!id||size!==buffer.byteLength-20||size>100000||v.getUint16(20)!==0xffd8)throw Error('JPEG invalido');
  return {id,age,service,size,jpeg:new Uint8Array(buffer,20)};
}
function cameraImageAge(frame,elapsed){return frame.age+Math.max(0,elapsed-frame.service);}
// At most 20 FPS, targeting <=80 kB/s of image payload. A larger JPEG slows
// requests instead of queuing more images or degrading the detection quality.
function cameraPeriod(previous,elapsed,bytes=0){return Math.max(50,bytes/80,Math.min(125,previous*0.8+elapsed*1.05*0.2));}
class CameraFeed {
  constructor(url,onFrame,onError,onMetrics){
    this.url=url;this.onFrame=onFrame;this.onError=onError;this.onMetrics=onMetrics;
    this.active=false;this.socket=null;this.pending=false;this.processing=false;this.epoch=0;this.period=50;
    this.timer=null;this.timeout=null;this.reconnect=null;this.failures=0;this.reconnections=0;
    this.bytes=0;this.windowAt=performance.now();this.failureRun=0;this.recoveryRun=0;this.lastId=0;
    this.ageSamples=0;this.ageTotal=0;this.ageMax=0;this.over100=0;
    this.canRequest=()=>true;this.requestBytes=new Uint8Array(5);this.requestBytes[0]=1;
  }
  start(){if(this.active)return;this.active=true;this.connect();}
  pause(){
    this.active=false;++this.epoch;clearTimeout(this.timer);clearTimeout(this.timeout);clearTimeout(this.reconnect);
    this.pending=false;this.processing=false;this.retireSocket();
  }
  retireSocket(){
    const socket=this.socket;this.socket=null;if(!socket)return;
    socket.onopen=socket.onmessage=socket.onclose=socket.onerror=null;
    try{socket.close();}catch{}
  }
  recover(message,epoch){
    if(!this.active||epoch!==this.epoch)return;
    ++this.epoch;++this.failures;++this.reconnections;++this.recoveryRun;
    clearTimeout(this.timeout);clearTimeout(this.timer);clearTimeout(this.reconnect);
    this.pending=false;this.processing=false;this.retireSocket();this.onError(message);
    // Don't wait for the browser's close handshake, which may never finish
    // promptly on a broken connection. Retired callbacks cannot affect new work.
    this.reconnect=setTimeout(()=>this.connect(),Math.min(3000,250*2**Math.min(4,this.recoveryRun-1)));
  }
  connect(){
    if(!this.active)return;
    const epoch=++this.epoch,socket=new WebSocket(this.url);this.socket=socket;socket.binaryType='arraybuffer';
    this.timeout=setTimeout(()=>this.recover('Camera: conexao nao abriu; reconectando',epoch),3000);
    socket.onopen=()=>{if(this.epoch===epoch){this.lastId=0;clearTimeout(this.timeout);this.request();}};
    socket.onmessage=async event=>{
      if(this.epoch!==epoch||!this.pending||this.processing)return;
      this.processing=true;
      clearTimeout(this.timeout);let failed=false,waiting=false;
      this.timeout=setTimeout(()=>this.recover('Camera: decodificacao travou; reconectando',epoch),1000);
      try{
        if(typeof event.data==='string'){
          let message='Captura indisponivel; nova tentativa';
          let parsed=null;try{parsed=JSON.parse(event.data);}catch{}
          if(parsed?.t==='wait'){waiting=true;return;}
          message=parsed?.message||message;
          throw Error(message);
        }
        const frame=parseCameraFrame(event.data);frame.started=this.started;
        if(frame.id===this.lastId){waiting=true;return;}
        await this.onFrame(frame,()=>this.active&&this.epoch===epoch);
        if(this.epoch!==epoch)return;
        this.lastId=frame.id;this.failureRun=0;this.recoveryRun=0;
        const now=performance.now();this.period=cameraPeriod(this.period,now-this.started,frame.size+20);this.bytes+=frame.size+20;
        const measuredAge=cameraImageAge(frame,now-this.started);
        ++this.ageSamples;this.ageTotal+=measuredAge;this.ageMax=Math.max(this.ageMax,measuredAge);if(measuredAge>100)++this.over100;
        if(now-this.windowAt>=1000){this.onMetrics({kbps:this.bytes/(now-this.windowAt),failures:this.failures,reconnections:this.reconnections,
          ageMean:this.ageTotal/this.ageSamples,ageMax:this.ageMax,over100:this.over100,ageSamples:this.ageSamples});this.bytes=0;this.windowAt=now;}
      }catch(error){if(this.epoch===epoch){failed=true;++this.failures;++this.failureRun;this.onError(error.message);}}
      finally{
        if(this.epoch===epoch){clearTimeout(this.timeout);this.pending=false;this.processing=false;this.schedule(waiting?30:failed?Math.min(500,100*2**Math.min(3,this.failureRun-1)):Math.max(0,this.period-(performance.now()-this.started)));}
      }
    };
    socket.onclose=()=>this.recover('Canal da camera desconectado; reconectando',epoch);
    socket.onerror=()=>this.recover('Falha na conexao da camera; reconectando',epoch);
  }
  schedule(ms){clearTimeout(this.timer);if(this.active)this.timer=setTimeout(()=>this.request(),ms);}
  request(){
    if(!this.active||this.pending||this.socket?.readyState!==WebSocket.OPEN)return;
    if(!this.canRequest()){this.schedule(10);return;}
    if(this.socket.bufferedAmount){this.schedule(50);return;}
    this.pending=true;this.started=performance.now();const epoch=this.epoch;
    const request=this.requestBytes;new DataView(request.buffer).setUint32(1,this.lastId,true);
    try{this.socket.send(request);}catch{this.recover('Falha ao pedir imagem; reconectando',epoch);return;}
    // One request stays in flight during TCP retransmission. Reopening the
    // socket for every late frame amplified congestion; freshness is checked
    // independently by the driving logic, never by reconnecting the preview.
    this.timeout=setTimeout(()=>this.recover('Camera sem resposta; reconectando',epoch),5000);
  }
}
if(typeof module!=='undefined'&&module.exports)module.exports={parseCameraFrame,cameraImageAge,cameraPeriod,CameraFeed};
</script>
<script>
const $=id=>document.getElementById(id);
const vision=new LineVision();
const lineRecovery=new LineRecovery();
const processing=document.createElement('canvas');processing.width=160;processing.height=120;
const pctx=processing.getContext('2d',{willReadFrequently:true});
const ctx=$('view').getContext('2d');
let ws=null,device=null,mode='idle',token=0,sequence=0,stopping=false,armPending=false;
let controlOpenedAt=0,lastControlAt=0,controlReconnect=null;
let pending=null;
let lastResult=null,lastFrame=0,lastFrameAt=0,lastImageAge=Infinity,goodFrames=0,frameRate=0,lastFrameTime=0;
let lastSentFrame=0,lastRtt=0,lastUptime=null,lastBootId=null,lastPaint=0;
let highestDisplayedAge=0;
let eventHistory=[];
try{const saved=JSON.parse(sessionStorage.getItem('carrinho.events')||'[]');if(Array.isArray(saved))eventHistory=saved.filter(v=>typeof v==='string').slice(0,16);}catch{}
function appendEvent(text){const li=document.createElement('li');li.textContent=text;$('log').prepend(li);while($('log').children.length>16)$('log').lastChild.remove();}
for(const text of [...eventHistory].reverse())appendEvent(text);
function log(text){const entry=new Date().toLocaleTimeString()+' — '+text;appendEvent(entry);eventHistory.unshift(entry);eventHistory.length=Math.min(16,eventHistory.length);try{sessionStorage.setItem('carrinho.events',JSON.stringify(eventHistory));}catch{}}
function connected(){return ws?.readyState===WebSocket.OPEN;}
// Seven near rows retain the step-mode anchor; two upper rows help resolve forks.
// Follow the continuation closest to the incoming heading, not the widest branch.
const TRACK_CONFIG=Object.freeze({autoThreshold:true,threshold:100,roiTop:0.35,roiBottom:0.90,lookAhead:0.8});
const SWAP_MOTORS=false; // true only if ENA drives the right wheel instead of left.
function config(){return {...TRACK_CONFIG};}
function send(data){if(!connected()||ws.bufferedAmount>1024)return false;try{ws.send(JSON.stringify(data));return true;}catch{return false;}}
function startBlockReason(){
  if(!connected()||!device)return 'Aguardando conexão e estado do carrinho.';
  if(device.control!=='steps-v10.6')return 'Este painel exige o firmware 10.6 por pulsos.';
  if(!device.actuatorsReady)return 'Atuadores indisponíveis. Consulte o diagnóstico.';
  if(!device.cameraReady||!device.frameReady)return 'Câmera indisponível. Consulte o diagnóstico.';
  if(device.armed||mode!=='idle')return 'Carrinho em movimento ou controlado por outra sessão.';
  if(armPending)return 'Aguardando confirmação de início.';
  if(stopping)return 'Aguardando confirmação da parada.';
  if(document.hidden)return 'Mantenha esta página visível para iniciar.';
  if(!lastResult?.valid)return lastResult?'Início bloqueado: '+lastResult.reason+'. Confira o enquadramento e a área de leitura.':'Aguardando a primeira imagem da linha.';
  if(lastImageAge+performance.now()-lastFrameAt>=220)return 'Aguardando imagem recente.';
  if(goodFrames<3)return 'Confirmando linha estável: '+goodFrames+'/3 imagens.';
  if(frameRate<6)return 'Imagem lenta: são necessárias pelo menos 6 imagens/s para iniciar.';
  return '';
}
function canStart(){return startBlockReason()==='';}
function controls(){
  const blocked=startBlockReason();$('start').disabled=!!blocked;$('start').title=blocked||'Iniciar seguidor';
}
function localStop(message){mode='idle';token=0;pending=null;armPending=false;goodFrames=0;vision.reset();lineRecovery.reset();if(message){$('notice').textContent=message;log(message);}controls();}
async function httpStop(){
  try{const r=await fetch('/stop',{method:'POST',signal:AbortSignal.timeout(1000)});if(r.ok){updateDevice(await r.json());stopping=false;controls();}}
  catch{log('Parada sem confirmação pela rede. Desligue a alimentação dos motores se necessário.');}
}
function stop(message='Parada solicitada'){
  localStop(message);stopping=true;
  if(!send({t:'stop'}))httpStop();
  else setTimeout(()=>{if(stopping)httpStop();},400);
  controls();
}
function updateDevice(s){
  const reboot=(lastBootId!==null&&s.bootId!==undefined&&s.bootId!==lastBootId)||(lastUptime!==null&&s.uptimeMs<lastUptime);
  if(reboot){
    const reason={1:'ligação ou reset externo',3:'reinício por software',4:'exceção no programa',5:'watchdog de interrupção',6:'watchdog de tarefa',7:'watchdog',9:'queda de tensão detectada'}[s.resetReason]||'motivo '+s.resetReason;
    localStop('A ESP32 reiniciou: '+reason+'.');
  }
  if(s.bootId!==undefined)lastBootId=s.bootId;
  lastUptime=s.uptimeMs;device=s;
  if(mode!=='idle'&&(!s.armed||s.token!==token)){localStop('Carrinho parou: '+s.reason);}
  if(!s.armed)stopping=false;
  paintDevice();
  controls();
}
function paintDevice(){
  const s=device;if(!s)return;
  $('device').textContent='ESP32: '+s.ip+' · Rede própria · dispositivos: '+s.apClients+'\nCiclo: '+(s.phase||'parado')+' · Servo desativado\nÚltimo PWM informado (0–255), esquerda / direita: '+s.left+' / '+s.right+'\nGiro das rodas: não medido\nTempo ligado: '+Math.floor(s.uptimeMs/1000)+' s'+(s.resetReason===9?' · Último reset: queda de tensão':'')+(s.actuatorsReady?'':'\nFalha PWM: '+s.actuatorError)+(s.cameraReady?'':'\nFalha câmera: '+s.cameraError);
}
function reconnectControl(message,detail=''){
  if(controlReconnect!==null)return;
  const old=ws;ws=null;
  if(old){old.onopen=old.onmessage=old.onclose=old.onerror=null;try{old.close();}catch{}}
  localStop(message);if(detail)log(detail);
  device=null;stopping=false;controls();
  controlReconnect=setTimeout(()=>{controlReconnect=null;openSocket();},1000);
}
function openSocket(){
  const socket=new WebSocket('ws://'+(globalThis.CARRINHO_HOST||location.host)+'/ws');ws=socket;
  controlOpenedAt=lastControlAt=performance.now();
  socket.onopen=()=>{if(ws!==socket)return;lastControlAt=performance.now();stopping=false;send({t:'status'});log('Canal de comandos conectado.');};
  socket.onmessage=event=>{
    if(ws!==socket)return;
    let s;try{s=JSON.parse(event.data);}catch{return;}
    if(!s||!['error','armed','ack','servo','stopped','status'].includes(s.t))return;
    lastControlAt=performance.now();
    if(s.t==='error'){stop('ESP32: '+s.message);return;}
    if(s.t==='armed'){
      if(!armPending){stop('Inicio cancelado.');return;}
      if(s.mode!==1||s.control!=='steps-v10.6'){stop('Modo de movimento inválido. Recarregue o painel.');return;}
      armPending=false;token=s.token;mode='auto';sequence=0;lastSentFrame=0;pending=null;lineRecovery.reset();
      updateDevice(s);$('notice').textContent='Modo seguidor ativado; preparando comandos.';
      log('Ciclo iniciado: observar, mover por um instante e parar. Servo desativado.');sendAuto();return;
    }
    if(s.t==='ack'){
      if(pending&&s.seq===pending.seq){lastRtt=performance.now()-pending.at;pending=null;
        if(device){device.phase=s.phase;device.left=s.left;device.right=s.right;paintDevice();}
        if(lastRtt>220){stop('Comando demorou demais. A comunicação está atrasada.');return;}
        // A newer image may have arrived while the previous command was pending.
        if(mode==='auto')sendAuto();
      }
      return; // Compact acknowledgements do not contain device status.
    }
    if(s.t==='stopped')stopping=false;
    updateDevice(s);
  };
  socket.onclose=event=>{
    if(ws!==socket)return;
    const detail='Diagnóstico: fechamento '+event.code+'; fase='+mode+'; comando pendente='+(pending?.seq??'nenhum')+'; última imagem='+Math.round(lastImageAge+performance.now()-lastFrameAt)+' ms; ESP ligada='+Math.floor((device?.uptimeMs??0)/1000)+' s.';
    reconnectControl('Conexão de comandos perdida. Reconectando sem retomar movimento.',detail);
  };
  socket.onerror=()=>{};
}
function drive(left,right,duration,frame,line){
  if(pending||!token||mode==='idle')return;
  const seq=++sequence;
  if(send({t:'step',token,seq:seq,left,right,duration,frame,line}))pending={seq,at:performance.now()};
  else stop('Canal de comandos congestionado.');
}
function sendAuto(){
  if(mode!=='auto'||pending)return;
  if(lastFrame===lastSentFrame)return;
  if(!lastResult||lastImageAge+performance.now()-lastFrameAt>220){stop('Imagem ausente ou antiga.');return;}
  if(lineRecovery.expired(performance.now())){stop('Linha não confirmada dentro do prazo de segurança.');return;}
  const recovering=lineRecovery.lostAt!==null;
  const command=differentialStep(lastResult,recovering,SWAP_MOTORS);
  const line=lastResult.ambiguous?2:lastResult.valid&&Number.isFinite(lastResult.error)?1:0;
  $('notice').textContent=recovering?
    (lineRecovery.confirmed(performance.now())?'Aguardando confirmação da linha; sem novo pulso. ':'Leitura momentânea ausente; concluindo apenas o pulso atual. ')+
    Math.max(0,(CARRINHO_MOTION.lossMs-(performance.now()-lineRecovery.lostAt))/1000).toFixed(1)+' s':
    'Próximo passo: '+command.direction+' · '+command.duration+' ms · PWM E/D: '+command.left+'/'+command.right;
  lastSentFrame=lastFrame;drive(command.left,command.right,command.duration,lastFrame,line);
}
function arm(){
  if(!canStart())return;
  if(!connected()||mode!=='idle'||armPending||stopping||device?.armed)return;
  armPending=true;controls();
  if(!send({t:'arm',mode:'steps-v10.6'})){armPending=false;controls();return;}
  setTimeout(()=>{if(armPending)stop('Inicio sem confirmação.');},800);
}
function draw(bitmap,result){
  ctx.drawImage(bitmap,0,0,640,480);ctx.lineWidth=2;
  ctx.strokeStyle='#f4dc87';ctx.strokeRect(4,result.top*4,632,(result.bottom-result.top)*4);
  ctx.strokeStyle='rgba(255,255,255,.65)';ctx.beginPath();ctx.moveTo(320,result.top*4);ctx.lineTo(320,result.bottom*4);ctx.stroke();
  ctx.strokeStyle=result.valid?'#30ffaf':'#ff7d7d';ctx.fillStyle=ctx.strokeStyle;ctx.lineWidth=3;ctx.beginPath();
  result.points.forEach((p,i)=>{if(i)ctx.lineTo(p.x*4,p.y*4);else ctx.moveTo(p.x*4,p.y*4);});ctx.stroke();
  for(const p of result.points){ctx.beginPath();ctx.arc(p.x*4,p.y*4,4,0,Math.PI*2);ctx.fill();}
  if(result.valid){
    const x=Math.max(4,Math.min(636,result.target*4)),y=result.top*4;
    ctx.strokeStyle='#7dccff';ctx.fillStyle='#7dccff';ctx.beginPath();
    ctx.moveTo(result.near*4,result.points[0].y*4);ctx.lineTo(x,y);ctx.stroke();
    ctx.beginPath();ctx.arc(x,y,7,0,Math.PI*2);ctx.fill();
  }
}
function stats(){
  const now=performance.now();if(now-lastPaint<100)return;lastPaint=now;
  if(!document.hidden&&lastFrame&&Number.isFinite(lastImageAge))highestDisplayedAge=Math.max(highestDisplayedAge,lastImageAge+now-lastFrameAt);
  $('fps').textContent=frameRate?frameRate.toFixed(1):'—';$('age').textContent=Number.isFinite(lastImageAge)?Math.round(lastImageAge+now-lastFrameAt)+' ms':'—';$('rtt').textContent=lastRtt?Math.round(lastRtt)+' ms':'—';
  if(lastResult)$('visionState').textContent=lastResult.reason+' · limiar '+lastResult.threshold+' · confiança '+Math.round(lastResult.confidence*100)+'%';
  if(lastResult)$('pathState').textContent=lastResult.valid?'Caminho: '+lastResult.direction+' · '+(lastResult.error>0.06?'corrigir à direita':lastResult.error< -0.06?'corrigir à esquerda':'manter o centro'):'Caminho: não identificado';
  if(mode==='idle'&&!armPending&&!stopping&&connected()&&device){
    $('notice').textContent=startBlockReason()||(lastResult?.limitedNear?'Linha estável. Pronto para iniciar.':'Linha estável. Pronto para iniciar.');
  }
  controls();
}
async function processFrame(frame,isCurrent){
  let bitmap=null;
  try{
    bitmap=await createImageBitmap(new Blob([frame.jpeg],{type:'image/jpeg'}));
    if(document.hidden||!isCurrent())return;
    // Always refresh the preview. A late image cannot drive the car: sendAuto
    // and the firmware independently enforce freshness. Do not reset transport.
    pctx.drawImage(bitmap,0,0,160,120);const rgba=pctx.getImageData(0,0,160,120).data;
    const result=vision.detect(rgba,160,120,config());const now=performance.now();
    // RTT includes capture time. Subtract server work before adding network
    // and decode time to the image's age at transmission; no double counting.
    lastImageAge=cameraImageAge(frame,now-frame.started);lastFrame=frame.id;lastFrameAt=now;lastResult=result;
    if(lastFrameTime){const rate=1000/(now-lastFrameTime);frameRate=frameRate?frameRate*0.75+rate*0.25:rate;}lastFrameTime=now;
    goodFrames=result.valid&&lastImageAge<220?goodFrames+1:0;
    if(mode==='auto'){
      lineRecovery.observe(result,now);sendAuto();
    }
    draw(bitmap,result); // Send steering before spending time drawing the preview.
    stats();
  }finally{
    bitmap?.close();
  }
}
const cameraFeed=new CameraFeed('ws://'+location.hostname+':81/camera',processFrame,message=>{
  goodFrames=0;
  $('cameraNetwork').textContent='Câmera: '+message;
  // A single missed capture can recover within the existing watchdog window.
  // No duplicate drive command is sent using the old image.
  controls();
},m=>{
  $('cameraNetwork').textContent='Imagem: '+m.kbps.toFixed(1)+' kB/s · falhas: '+m.failures+' · reconexões: '+m.reconnections+
    ' · Idade ao processar: média '+Math.round(m.ageMean)+' ms / pico '+Math.round(m.ageMax)+' ms · acima de 100 ms: '+m.over100+'/'+m.ageSamples+
    ' · Pico entre atualizações: '+Math.round(highestDisplayedAge)+' ms';
});
cameraFeed.canRequest=()=>!pending&&!armPending&&!stopping;
$('start').onclick=()=>arm();$('stop').onclick=()=>stop();
setInterval(()=>{
  const now=performance.now();
  stats();
  // A half-open TCP connection need not produce onclose promptly. Retire it
  // with stale callbacks detached; reconnect only for observation, never arm.
  if(ws?.readyState===0&&now-controlOpenedAt>3000){reconnectControl('Canal de comandos não abriu. Reconectando.');return;}
  if(connected()&&now-lastControlAt>(stopping?1500:4500)){reconnectControl('Canal de comandos sem resposta. Reconectando sem retomar movimento.');return;}
  if(mode!=='idle'&&(document.hidden||!connected()||(pending&&now-pending.at>250))){stop('Sem confirmação recente dos comandos.');return;}
  if(mode==='auto'&&lineRecovery.expired(now)){stop('Linha ausente por 3 segundos.');return;}
  if(mode==='auto'&&(now-lastFrameAt>250||lastImageAge+now-lastFrameAt>280)){stop('Imagem deixou de atualizar.');return;}
  if(now-lastFrameAt>500){goodFrames=0;frameRate=0;lastFrameTime=0;$('visionState').textContent='Aguardando nova imagem…';$('pathState').textContent='Caminho: sem imagem recente';controls();}
},20);
setInterval(()=>{if(connected()&&!pending&&!armPending)send({t:'status'});},3000);
document.addEventListener('visibilitychange',()=>{if(document.hidden){stop('Página ficou em segundo plano.');cameraFeed.pause();}else{goodFrames=0;vision.reset();cameraFeed.start();}});
window.addEventListener('pagehide',()=>{send({t:'stop'});cameraFeed.pause();navigator.sendBeacon('/stop','');});
openSocket();if(!document.hidden)cameraFeed.start();

</script>
</body></html>

)LINEPAGE";

bool networkReady(){return apReady.load()&&apHasClient.load();}

bool setDuty(ledc_channel_t channel,uint32_t duty){
  esp_err_t e=ledc_set_duty(PWM_MODE,channel,duty);
  if(e==ESP_OK)e=ledc_update_duty(PWM_MODE,channel);
  if(e!=ESP_OK){portENTER_CRITICAL(&stateMux);actuatorError=e;actuatorsReady=false;drive.stop(DriveControl::FAULT);portEXIT_CRITICAL(&stateMux);}
  return e==ESP_OK;
}

esp_err_t setupPwmTimer(ledc_timer_t number,int hz,ledc_timer_bit_t bits){
  ledc_timer_config_t t={};t.speed_mode=PWM_MODE;t.timer_num=number;
  t.freq_hz=hz;t.duty_resolution=bits;t.clk_cfg=LEDC_AUTO_CLK;
  return ledc_timer_config(&t);
}
esp_err_t setupPwmPin(int pin,ledc_channel_t channel,ledc_timer_t timer){
  ledc_channel_config_t c={};c.gpio_num=pin;c.speed_mode=PWM_MODE;c.channel=channel;
  c.timer_sel=timer;c.duty=0;c.hpoint=0;c.intr_type=LEDC_INTR_DISABLE;
  return ledc_channel_config(&c);
}

void actuatorTask(void*){
  TickType_t wake=xTaskGetTickCount();
  int oldLeft=-1,oldRight=-1;
  for(;;){
    const bool connected=networkReady();
    int left,right;bool ok;
    portENTER_CRITICAL(&stateMux);
    drive.tick(millis(),connected);ok=actuatorsReady;
    left=ok?static_cast<int>(drive.outputLeft+0.5f):0;
    right=ok?static_cast<int>(drive.outputRight+0.5f):0;
    portEXIT_CRITICAL(&stateMux);
    // Unica tarefa que escreve o PWM em operacao; nenhuma fila de movimentos.
    if(left!=oldLeft){if(setDuty(LEFT_CH,left))oldLeft=left;}
    if(right!=oldRight){if(setDuty(RIGHT_CH,right))oldRight=right;}
    vTaskDelayUntil(&wake,pdMS_TO_TICKS(5)); // Libera CPU; nao bloqueia rede ou camera.
  }
}

void initActuators(){
  // No servo timer/channel is attached, even if an old page sends a servo command.
  pinMode(SERVO_PIN,INPUT_PULLDOWN);
  digitalWrite(ENA_PIN,LOW);digitalWrite(ENB_PIN,LOW);
  pinMode(ENA_PIN,OUTPUT);pinMode(ENB_PIN,OUTPUT);
  digitalWrite(ENA_PIN,LOW);digitalWrite(ENB_PIN,LOW);
  actuatorError=setupPwmTimer(LEDC_TIMER_1,1000,LEDC_TIMER_8_BIT);
  if(actuatorError==ESP_OK)actuatorError=setupPwmPin(ENA_PIN,LEFT_CH,LEDC_TIMER_1);
  if(actuatorError==ESP_OK)actuatorError=setupPwmPin(ENB_PIN,RIGHT_CH,LEDC_TIMER_1);
  actuatorsReady=actuatorError==ESP_OK;
  if(actuatorsReady&&xTaskCreatePinnedToCore(actuatorTask,"actuators",4096,nullptr,4,nullptr,1)!=pdPASS){
    actuatorsReady=false;actuatorError=ESP_ERR_NO_MEM;
  }
}

bool initCamera(){
  camera_config_t c={};c.ledc_channel=LEDC_CHANNEL_0;c.ledc_timer=LEDC_TIMER_0;
  c.pin_d0=5;c.pin_d1=18;c.pin_d2=19;c.pin_d3=21;c.pin_d4=36;c.pin_d5=39;c.pin_d6=34;c.pin_d7=35;
  c.pin_xclk=0;c.pin_pclk=22;c.pin_vsync=25;c.pin_href=23;c.pin_sccb_sda=26;c.pin_sccb_scl=27;
  c.pin_pwdn=32;c.pin_reset=-1;c.xclk_freq_hz=16000000;c.pixel_format=PIXFORMAT_JPEG;
  // Keep the larger JPEG allocation to avoid FB_OVF, then output QVGA.
  // Two JPEG buffers in PSRAM overlap sensor capture and consumption.
  // LATEST with two buffers keeps only one completed frame in the driver queue.
  // The separate latest-frame cache returns driver buffers before network sends.
  c.frame_size=FRAMESIZE_VGA;
  const bool hasPsram=psramFound();
  c.jpeg_quality=18;c.fb_count=hasPsram&&CAMERA_BUFFER_DUPLO?2:1;
  c.fb_location=hasPsram?CAMERA_FB_IN_PSRAM:CAMERA_FB_IN_DRAM;
  c.grab_mode=c.fb_count>1?CAMERA_GRAB_LATEST:CAMERA_GRAB_WHEN_EMPTY;
  cameraDriverBuffers=0;
  cameraError=esp_camera_init(&c);
  if(cameraError!=ESP_OK&&c.fb_count>1){
    Serial.println("CAM: captura dupla indisponivel; tentando um buffer.");
    esp_camera_deinit();c.fb_count=1;c.grab_mode=CAMERA_GRAB_WHEN_EMPTY;
    cameraError=esp_camera_init(&c);
  }
  if(cameraError!=ESP_OK)return false;
  sensor_t *sensor=esp_camera_sensor_get();
  if(!sensor||sensor->set_framesize(sensor,FRAMESIZE_QVGA)!=0){
    cameraError=ESP_FAIL;esp_camera_deinit();return false;
  }
  cameraDriverBuffers=c.fb_count;
  // Sensor grayscale removes color detail before JPEG, without ESP32 conversion.
  const bool grayscale=sensor->set_special_effect&&sensor->set_special_effect(sensor,2)==0;
  if(!grayscale)Serial.println("Camera: cinza indisponivel; mantendo JPEG em cores.");
  // OV5640 night mode may extend exposure/frame duration. Keep automatic
  // exposure but disable that extension; don't force undocumented timings.
  if(sensor->id.PID==0x5640&&sensor->set_aec2&&sensor->set_aec2(sensor,0)!=0)
    Serial.println("Camera: nao foi possivel desativar modo noturno.");
  for(int i=0;i<c.fb_count+1;i++){
    camera_fb_t *warmup=esp_camera_fb_get();
    if(warmup)esp_camera_fb_return(warmup);
  }
  Serial.printf("Camera V10.6: JPEG 320x240 Q18 | %s | buffer VGA | %d buffer(s) | PSRAM=%s | sensor=0x%04x\n",
                grayscale?"cinza":"cores",int(c.fb_count),hasPsram?"sim":"nao",sensor->id.PID);
  return true;
}

String stateJson(const char *type="status",uint32_t seq=0){
  bool armed,ready;int left,right,mode;uint32_t token,age,remaining;const char *reason,*phase;
  esp_err_t pwmError;
  portENTER_CRITICAL(&stateMux);
  armed=drive.armed;ready=actuatorsReady;phase=drive.phaseText();
  left=static_cast<int>(drive.outputLeft);right=static_cast<int>(drive.outputRight);
  token=drive.token;age=drive.commandAge(millis());remaining=drive.remaining(millis());
  mode=drive.mode;reason=drive.reasonText();pwmError=actuatorError;
  portEXIT_CRITICAL(&stateMux);
  char json[768];
  snprintf(json,sizeof(json),
    "{\"t\":\"%s\",\"seq\":%lu,\"armed\":%s,\"token\":%lu,\"mode\":%d,\"reason\":\"%s\","
    "\"actuatorsReady\":%s,\"actuatorError\":\"%s\",\"cameraReady\":%s,\"cameraError\":\"%s\",\"frameReady\":%s,"
    "\"control\":\"steps-v10.6\",\"phase\":\"%s\",\"servoEnabled\":false,\"left\":%d,\"right\":%d,\"commandAge\":%lu,\"remainingMs\":%lu,"
    "\"uptimeMs\":%lu,\"bootId\":%lu,\"resetReason\":%d,\"apClients\":%d,\"ip\":\"%s\"}",
    type,(unsigned long)seq,armed?"true":"false",(unsigned long)token,mode,reason,
    ready?"true":"false",esp_err_to_name(pwmError),cameraReady?"true":"false",esp_err_to_name(cameraError),frameServer?"true":"false",
    phase,left,right,(unsigned long)age,(unsigned long)remaining,
    (unsigned long)millis(),(unsigned long)bootId,static_cast<int>(esp_reset_reason()),int(WiFi.softAPgetStationNum()),WiFi.softAPIP().toString().c_str());
  return String(json);
}

esp_err_t wsSend(httpd_req_t *req,const String &message){
  // One complete WS frame per send, including short status/ACK/WAIT replies.
  // The IDF helper splits header and payload into separate TCP writes.
  const size_t length=message.length();
  if(length>1024)return ESP_FAIL;
  uint8_t packet[1028];packet[0]=0x81; // FIN + text, server frames are unmasked.
  const size_t header=length<126?2:4;
  if(header==2)packet[1]=static_cast<uint8_t>(length);
  else {packet[1]=126;packet[2]=length>>8;packet[3]=length;}
  memcpy(packet+header,message.c_str(),length);
  const int sent=controlSendAll(req->handle,httpd_req_to_sockfd(req),reinterpret_cast<const char*>(packet),length+header,0);
  return sent==static_cast<int>(length+header)?ESP_OK:ESP_FAIL;
}
esp_err_t wsError(httpd_req_t *req,const char *message){
  return wsSend(req,String("{\"t\":\"error\",\"message\":\"")+message+"\"}");
}
int controlSendAll(httpd_handle_t handle,int socket,const char *data,size_t length,int flags);
void freeClient(void *context){
  ClientSession *client=static_cast<ClientSession*>(context);
  if(!client)return;
  ++controlClosed;
  portENTER_CRITICAL(&stateMux);
  const bool owner=drive.armed&&drive.owner==client->id;
  if(owner)drive.stop(DriveControl::NO_COMMAND);
  portEXIT_CRITICAL(&stateMux);
  Serial.printf("CTRL canal encerrado: dono_ativo=%d tempo=%lu\n",owner,(unsigned long)millis());
  delete client;
}
ClientSession *ensureClientSession(httpd_req_t *req){
  ClientSession *client=static_cast<ClientSession*>(req->sess_ctx);
  if(client)return client;
  client=new(std::nothrow) ClientSession;
  if(!client)return nullptr;
  client->id=esp_random();if(!client->id)client->id=1;
  req->sess_ctx=client;req->free_ctx=freeClient;
  const int enabled=1;
  setsockopt(httpd_req_to_sockfd(req),IPPROTO_TCP,TCP_NODELAY,&enabled,sizeof(enabled));
  return client;
}
bool numberField(cJSON *json,const char *name,uint32_t max,uint32_t &result){
  cJSON *item=cJSON_GetObjectItemCaseSensitive(json,name);
  if(!cJSON_IsNumber(item)||!isfinite(item->valuedouble)||item->valuedouble<0||
      item->valuedouble>max||floor(item->valuedouble)!=item->valuedouble)return false;
  result=static_cast<uint32_t>(item->valuedouble);return true;
}

esp_err_t wsHandler(httpd_req_t *req){
  // IDF's WS sender accepts partial writes as success. Install before the
  // first response, including "armed"; never continue a truncated stream.
  if(!req->sess_ctx){
    const esp_err_t installed=httpd_sess_set_send_override(req->handle,httpd_req_to_sockfd(req),controlSendAll);
    if(installed!=ESP_OK)return installed;
  }
  // IDF 5.5.5 skips this handler during the handshake. Initialize on the
  // first data frame too, while retaining compatibility with older cores.
  ClientSession *client=ensureClientSession(req);
  if(!client)return ESP_ERR_NO_MEM;
  if(req->method==HTTP_GET)return ESP_OK;
  httpd_ws_frame_t frame={};
  esp_err_t error=httpd_ws_recv_frame(req,&frame,0);
  if(error!=ESP_OK)return error;
  if(frame.type!=HTTPD_WS_TYPE_TEXT||frame.len==0||frame.len>384)return ESP_FAIL;
  uint8_t payload[385]={};frame.payload=payload;
  error=httpd_ws_recv_frame(req,&frame,384);if(error!=ESP_OK)return error;
  cJSON *json=cJSON_Parse(reinterpret_cast<const char*>(payload));
  if(!json)return wsError(req,"Comando invalido");
  cJSON *type=cJSON_GetObjectItemCaseSensitive(json,"t");
  if(!cJSON_IsString(type)){cJSON_Delete(json);return wsError(req,"Tipo de comando ausente");}
  const char *t=type->valuestring;
  const uint32_t session=client->id;
  const bool connected=networkReady();
  const uint32_t now=millis();
  const char *reply="status";const char *failure=nullptr;uint32_t seq=0;
  if(strcmp(t,"stop")==0||strcmp(t,"release")==0){
    portENTER_CRITICAL(&stateMux);drive.stop(DriveControl::USER_STOP);
    portEXIT_CRITICAL(&stateMux);reply="stopped";
  }else if(strcmp(t,"arm")==0){
    cJSON *m=cJSON_GetObjectItemCaseSensitive(json,"mode");
    const bool isAuto=cJSON_IsString(m)&&strcmp(m->valuestring,"steps-v10.6")==0;
    uint32_t token=esp_random();if(!token)token=1;
    bool ok=false;
    portENTER_CRITICAL(&stateMux);
    const bool fresh=nextFrame&&static_cast<uint32_t>(now-frameStamps[nextFrame%8].captured)<=DriveControl::FRAME_MAX_AGE_MS;
    if(actuatorsReady&&connected&&isAuto&&cameraReady&&frameServer&&fresh)
      ok=drive.arm(now,token,session,DriveControl::AUTO_LINE);
    portEXIT_CRITICAL(&stateMux);
    Serial.printf("CTRL iniciar: aceito=%d tempo=%lu\n",ok,(unsigned long)now);
    if(!ok)failure="Nao iniciou: modo invalido, sessao ativa, atuadores indisponiveis ou imagem antiga";
    else reply="armed";
  }else if(strcmp(t,"step")==0){
    uint32_t token=0,left=0,right=0,duration=0,id=0,line=0;
    const bool valid=numberField(json,"token",UINT32_MAX,token)&&numberField(json,"seq",UINT32_MAX,seq)&&
      numberField(json,"left",DriveControl::MAX_PWM,left)&&numberField(json,"right",DriveControl::MAX_PWM,right)&&
      numberField(json,"duration",DriveControl::MAX_STEP_MS,duration)&&duration>=60&&numberField(json,"frame",UINT32_MAX,id)&&numberField(json,"line",2,line);
    bool ok=false,stopped=false;uint32_t frameAge=UINT32_MAX;
    portENTER_CRITICAL(&stateMux);
    for(const auto &stamp:frameStamps)if(id&&stamp.id==id){frameAge=static_cast<uint32_t>(now-stamp.captured);break;}
    if(valid&&actuatorsReady&&connected)ok=drive.command(now,token,session,seq,left,right,duration,id,frameAge,line);
    stopped=!drive.armed;
    portEXIT_CRITICAL(&stateMux);
    if(seq==1||!ok)Serial.printf("CTRL passo: seq=%lu aceito=%d pwm=%lu/%lu duracao=%lu imagem=%lu idade=%lu\n",
      (unsigned long)seq,ok,(unsigned long)left,(unsigned long)right,(unsigned long)duration,(unsigned long)id,(unsigned long)frameAge);
    if(!ok&&valid&&stopped)reply="stopped";
    else if(!ok)failure="Comando rejeitado: prazo, sessao, imagem ou valores invalidos";
    else reply="ack";
  }else if(strcmp(t,"servo")==0){
    failure="Servo desativado nesta versao";
  }else if(strcmp(t,"status")!=0)failure="Comando desconhecido";
  cJSON_Delete(json);
  if(!failure&&strcmp(reply,"ack")==0){
    const char *phase;int left,right;
    portENTER_CRITICAL(&stateMux);
    phase=drive.phaseText();left=static_cast<int>(drive.outputLeft+0.5f);right=static_cast<int>(drive.outputRight+0.5f);
    portEXIT_CRITICAL(&stateMux);
    char ack[128];snprintf(ack,sizeof(ack),"{\"t\":\"ack\",\"seq\":%lu,\"phase\":\"%s\",\"left\":%d,\"right\":%d}",(unsigned long)seq,phase,left,right);
    return wsSend(req,String(ack));
  }
  return failure?wsError(req,failure):wsSend(req,stateJson(reply,seq));
}

esp_err_t closeHttpResponse(httpd_req_t *req,esp_err_t result){
  httpd_sess_trigger_close(req->handle,httpd_req_to_sockfd(req));
  return result;
}
esp_err_t pageHandler(httpd_req_t *req){
  httpd_resp_set_hdr(req,"Connection","close");
  httpd_resp_set_type(req,"text/html; charset=utf-8");httpd_resp_set_hdr(req,"Cache-Control","no-store");
  return closeHttpResponse(req,httpd_resp_send(req,PAGE_HTML,HTTPD_RESP_USE_STRLEN));
}
esp_err_t statusHandler(httpd_req_t *req){
  httpd_resp_set_hdr(req,"Connection","close");
  const String json=stateJson();httpd_resp_set_type(req,"application/json");httpd_resp_set_hdr(req,"Cache-Control","no-store");
  return closeHttpResponse(req,httpd_resp_send(req,json.c_str(),json.length()));
}
esp_err_t stopHandler(httpd_req_t *req){
  portENTER_CRITICAL(&stateMux);drive.stop(DriveControl::USER_STOP);portEXIT_CRITICAL(&stateMux);
  return statusHandler(req);
}

// httpd_ws_send_frame() in IDF 5.5.5 treats a short positive send as success.
// Complete every byte, including WS headers, or fail and close the session.
int socketSendAll(int socket,const char *data,size_t length,int flags,uint32_t timeoutMs){
  const uint32_t started=millis();size_t offset=0;unsigned writes=0;
  while(offset<length){
    if(static_cast<uint32_t>(millis()-started)>=timeoutMs)return HTTPD_SOCK_ERR_TIMEOUT;
    const size_t chunk=(length-offset)>1460?1460:(length-offset);
    const int sent=send(socket,data+offset,chunk,flags|MSG_DONTWAIT);
    if(sent>0){
      offset+=static_cast<size_t>(sent);
      // Even successful short writes must not form a CPU-consuming loop.
      if(++writes%4==0&&offset<length)vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }
    if(sent==0)return HTTPD_SOCK_ERR_FAIL;
    if(errno!=EAGAIN&&errno!=EWOULDBLOCK&&errno!=EINTR)return HTTPD_SOCK_ERR_FAIL;
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  return static_cast<int>(offset);
}
int cameraSendAll(httpd_handle_t,int socket,const char *data,size_t length,int flags){
  const uint32_t started=millis();
  const int result=socketSendAll(socket,data,length,flags,1000);
  const uint32_t elapsed=millis()-started;cameraSendLastMs=elapsed;
  if(elapsed>cameraSendMaxMs.load())cameraSendMaxMs=elapsed;
  if(result<0)++cameraSendErrors;
  return result;
}
int controlSendAll(httpd_handle_t,int socket,const char *data,size_t length,int flags){
  // Header and payload each have a bounded retry budget; the actuator task
  // remains independent and retains its 300 ms command watchdog.
  const uint32_t started=millis();
  const int result=socketSendAll(socket,data,length,flags,100);
  controlSendLastMs=millis()-started;if(result<0)++controlSendErrors;
  if(result<0)Serial.printf("CTRL envio: erro=%d tempo=%lu\n",result,(unsigned long)millis());
  return result;
}

void cameraFailureLog(const char *phase,esp_err_t error,uint32_t elapsed){
  static uint32_t lastLog=0,failures=0;
  const uint32_t now=millis();uint32_t count;bool emit;
  portENTER_CRITICAL(&stateMux);
  count=++failures;emit=count==1||static_cast<uint32_t>(now-lastLog)>=2000;
  if(emit)lastLog=now;
  portEXIT_CRITICAL(&stateMux);
  if(emit){
    Serial.printf("CAM %s: erro=%d tempo=%lu ms falhas=%lu heap=%u\n",phase,int(error),
                  (unsigned long)elapsed,(unsigned long)count,unsigned(ESP.getFreeHeap()));
  }
}

camera_fb_t *acquireFreshFrame(uint32_t &captured,uint32_t &age){
  if(!cameraReady)return nullptr;
  const uint32_t started=millis();
  for(int attempt=0;attempt<2;++attempt){
    camera_fb_t *fb=esp_camera_fb_get();
    if(!fb)return nullptr;
    captured=static_cast<uint32_t>(uint64_t(fb->timestamp.tv_sec)*1000+fb->timestamp.tv_usec/1000);
    age=static_cast<uint32_t>(millis()-captured);
    if(age<=250&&fb->format==PIXFORMAT_JPEG&&fb->len>=4&&
       fb->buf[0]==0xff&&fb->buf[1]==0xd8)return fb;
    esp_camera_fb_return(fb);
    if(static_cast<uint32_t>(millis()-started)>=250)break;
  }
  return nullptr;
}

uint32_t registerFrame(uint32_t captured){
  portENTER_CRITICAL(&stateMux);
  uint32_t id=++nextFrame;if(!id)id=++nextFrame;
  frameStamps[id%8].id=id;frameStamps[id%8].captured=captured;
  portEXIT_CRITICAL(&stateMux);
  return id;
}

void captureTask(void*){
  uint32_t failures=0,lastRecovery=millis();
  for(;;){
    const uint32_t started=millis();
    // No live viewer: stop draining/copying frames; continuous sensor mode may still run.
    if(static_cast<uint32_t>(started-cameraDemandAt.load())>2000){
      vTaskDelay(pdMS_TO_TICKS(50));continue;
    }
    uint32_t captured=0,age=0;
    camera_fb_t *fb=acquireFreshFrame(captured,age);captureLastMs=millis()-started;
    if(fb){
      failures=0;cameraLastBytes=fb->len;captureAgeAtReadMs=millis()-captured;
      int slot=-1;
      if(fb->len<=cameraCacheCapacity){
        portENTER_CRITICAL(&stateMux);slot=cameraCache.reserveWrite();portEXIT_CRITICAL(&stateMux);
      }
      if(slot>=0){
        memcpy(cameraCache.slots[slot].data+CAMERA_PREFIX,fb->buf,fb->len);
        const size_t length=fb->len;
        esp_camera_fb_return(fb); // Release DMA buffer before any network work.
        const uint32_t id=registerFrame(captured);
        portENTER_CRITICAL(&stateMux);cameraCache.publish(slot,length,captured,id);portEXIT_CRITICAL(&stateMux);
      }else {++captureDrops;esp_camera_fb_return(fb);}
    }else{
      ++failures;++captureFailures;cameraFailureLog("CAPTURA",ESP_FAIL,millis()-started);
      if(failures>=3&&static_cast<uint32_t>(millis()-lastRecovery)>=10000){
        portENTER_CRITICAL(&stateMux);cameraReady=false;cameraCache.invalidate();drive.stop(DriveControl::LINE_LOST);portEXIT_CRITICAL(&stateMux);
        Serial.println("CAM: recuperando sensor; movimento permanece parado.");
        esp_camera_deinit();const bool ready=initCamera();
        portENTER_CRITICAL(&stateMux);cameraReady=ready;portEXIT_CRITICAL(&stateMux);
        lastRecovery=millis();failures=0;
      }
    }
    const uint32_t spent=millis()-started;
    // Drain continuous capture promptly. No queue of JPEGs for transmission.
    // Fallback retains the 10.5 capture schedule; always yield to other tasks.
    const uint32_t period=cameraDriverBuffers.load()>1?25:40;
    vTaskDelay(pdMS_TO_TICKS(spent<period?period-spent:5));
  }
}

bool startCameraPipeline(){
  cameraCacheCapacity=psramFound()?65536:24576;
  const uint32_t caps=MALLOC_CAP_8BIT|(psramFound()?MALLOC_CAP_SPIRAM:MALLOC_CAP_INTERNAL);
  for(auto &slot:cameraCache.slots)slot.data=static_cast<uint8_t*>(heap_caps_malloc(cameraCacheCapacity+CAMERA_PREFIX,caps));
  if(cameraCache.slots[0].data&&cameraCache.slots[1].data&&
     xTaskCreatePinnedToCore(captureTask,"capture",6144,nullptr,2,nullptr,1)==pdPASS)return true;
  for(auto &slot:cameraCache.slots){heap_caps_free(slot.data);slot.data=nullptr;}
  cameraError=ESP_ERR_NO_MEM;return false;
}

int acquireCachedFrame(uint32_t seen){
  cameraDemandAt=millis();
  portENTER_CRITICAL(&stateMux);const int slot=cameraCache.acquire(millis(),seen);portEXIT_CRITICAL(&stateMux);
  return slot;
}
void releaseCachedFrame(int slot){
  portENTER_CRITICAL(&stateMux);cameraCache.release(slot);portEXIT_CRITICAL(&stateMux);
}

// One binary request (action + last ID) -> one binary JPEG response. Separate server/task
// from controls. No loop pushing frames and no queue of images to transmit.
esp_err_t cameraWsHandler(httpd_req_t *req){
  // Apply on data requests as well: newer cores skip the handshake callback.
  const int enabled=1;
  setsockopt(httpd_req_to_sockfd(req),IPPROTO_TCP,TCP_NODELAY,&enabled,sizeof(enabled));
  if(httpd_sess_set_send_override(req->handle,httpd_req_to_sockfd(req),cameraSendAll)!=ESP_OK)return ESP_FAIL;
  if(req->method==HTTP_GET)return ESP_OK;
  const uint32_t started=millis();
  httpd_ws_frame_t request={};
  esp_err_t e=httpd_ws_recv_frame(req,&request,0);
  if(e!=ESP_OK)return e;
  if(request.type!=HTTPD_WS_TYPE_BINARY||request.len!=5)return ESP_FAIL;
  uint8_t action[5]={};request.payload=action;
  e=httpd_ws_recv_frame(req,&request,sizeof(action));
  if(e!=ESP_OK)return e;
  if(action[0]!=1)return ESP_FAIL;
  const uint32_t seen=uint32_t(action[1])|(uint32_t(action[2])<<8)|(uint32_t(action[3])<<16)|(uint32_t(action[4])<<24);
  int slot=acquireCachedFrame(seen);
  while(slot<0&&cameraReady&&networkReady()&&static_cast<uint32_t>(millis()-started)<60){
    vTaskDelay(pdMS_TO_TICKS(5));slot=acquireCachedFrame(seen);
  }
  if(slot<0){++cameraWaits;return wsSend(req,String("{\"t\":\"wait\"}"));}
  const auto &fb=cameraCache.slots[slot];
  const uint32_t now=millis();cameraAgeAtSendMs=now-fb.captured;
  // Header is explicitly little endian: magic, id, age, service time, JPEG size.
  const uint32_t values[]={0x374d4143,fb.id,static_cast<uint32_t>(now-fb.captured),static_cast<uint32_t>(now-started),static_cast<uint32_t>(fb.len)};
  uint8_t *metadata=fb.data+10;
  for(int i=0;i<5;++i)for(int j=0;j<4;++j)metadata[i*4+j]=static_cast<uint8_t>(values[i]>>(8*j));
  // One complete, unmasked server WebSocket frame, including its header.
  // Space is reserved in the cache: no per-frame allocation or second JPEG copy.
  // Coalescing the headers with JPEG data avoids four tiny NODELAY writes.
  const size_t payloadLength=20+fb.len;
  const size_t headerLength=payloadLength<126?2:payloadLength<=65535?4:10;
  uint8_t *packet=metadata-headerLength;packet[0]=0x82; // FIN + binary
  if(headerLength==2)packet[1]=static_cast<uint8_t>(payloadLength);
  else if(headerLength==4){packet[1]=126;packet[2]=payloadLength>>8;packet[3]=payloadLength;}
  else {packet[1]=127;for(int i=0;i<8;++i)packet[2+i]=static_cast<uint8_t>(uint64_t(payloadLength)>>(56-8*i));}
  const size_t packetLength=headerLength+payloadLength;
  const int written=cameraSendAll(req->handle,httpd_req_to_sockfd(req),reinterpret_cast<const char*>(packet),packetLength,0);
  e=written==static_cast<int>(packetLength)?ESP_OK:ESP_FAIL;
  releaseCachedFrame(slot);
  if(e!=ESP_OK)cameraFailureLog("ENVIO",e,millis()-started);else ++cameraSent;
  return e;
}

// Snapshot endpoint retained for diagnostics, not used by the live panel.
esp_err_t frameHandler(httpd_req_t *req){
  httpd_resp_set_hdr(req,"Connection","close");
  httpd_resp_set_hdr(req,"Access-Control-Allow-Origin","*");
  httpd_resp_set_hdr(req,"Access-Control-Expose-Headers","X-Frame-Id, X-Frame-Age");
  httpd_resp_set_hdr(req,"Cache-Control","no-store");
  if(!cameraReady){httpd_resp_set_status(req,"503 Service Unavailable");return closeHttpResponse(req,httpd_resp_sendstr(req,"Camera indisponivel"));}
  const int slot=acquireCachedFrame(0);
  if(slot<0){
    httpd_resp_set_status(req,"503 Service Unavailable");
    return closeHttpResponse(req,httpd_resp_sendstr(req,"Falha de captura ou imagem antiga. Confira o Serial."));
  }
  const auto &fb=cameraCache.slots[slot];
  char idText[16],ageText[16];snprintf(idText,sizeof(idText),"%lu",(unsigned long)fb.id);snprintf(ageText,sizeof(ageText),"%lu",(unsigned long)(millis()-fb.captured));
  httpd_resp_set_hdr(req,"X-Frame-Id",idText);httpd_resp_set_hdr(req,"X-Frame-Age",ageText);
  httpd_resp_set_type(req,"image/jpeg");
  const esp_err_t e=httpd_resp_send(req,reinterpret_cast<const char*>(fb.data+CAMERA_PREFIX),fb.len);
  releaseCachedFrame(slot);return closeHttpResponse(req,e);
}

void readSavedCrash(){
#if CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH && CONFIG_ESP_COREDUMP_DATA_FORMAT_ELF
  const esp_err_t checked=esp_core_dump_image_check();
  if(checked!=ESP_OK){snprintf(savedCrash,sizeof(savedCrash),"Registro salvo indisponivel: %s\n",esp_err_to_name(checked));return;}
  esp_core_dump_summary_t summary={};
  const esp_err_t result=esp_core_dump_get_summary(&summary);
  if(result!=ESP_OK){snprintf(savedCrash,sizeof(savedCrash),"Falha ao ler registro salvo: %s\n",esp_err_to_name(result));return;}
  char panic[160]={};
  if(esp_core_dump_get_panic_reason(panic,sizeof(panic))!=ESP_OK)snprintf(panic,sizeof(panic),"Motivo nao disponivel");
  // This is the last SAVED crash, possibly from an older boot/firmware.
  // Read once at boot, before starting camera/network; never erase the dump.
  size_t used=snprintf(savedCrash,sizeof(savedCrash),
    "ULTIMA FALHA SALVA (pode ser de uma inicializacao anterior)\nTarefa: %.15s\nPC: 0x%08lx\nMotivo: %.159s\nELF: %.*s\nBacktrace:",
    summary.exc_task,(unsigned long)summary.exc_pc,panic,int(sizeof(summary.app_elf_sha256)-1),reinterpret_cast<const char*>(summary.app_elf_sha256));
  for(unsigned i=0;i<summary.exc_bt_info.depth&&i<sizeof(summary.exc_bt_info.bt)/sizeof(summary.exc_bt_info.bt[0])&&used+12<sizeof(savedCrash);++i)
    used+=snprintf(savedCrash+used,sizeof(savedCrash)-used," %08lx",(unsigned long)summary.exc_bt_info.bt[i]);
  Serial.println(savedCrash);
#endif
}
esp_err_t networkDiagnosticHandler(httpd_req_t *req){
  wifi_sta_list_t stations={};const bool signalOK=esp_wifi_ap_get_sta_list(&stations)==ESP_OK&&stations.num>0;
  char signal[16];if(signalOK)snprintf(signal,sizeof(signal),"%d",int(stations.sta[0].rssi));else strcpy(signal,"null");
  const uint32_t now=millis();uint32_t frameAge=UINT32_MAX;
  portENTER_CRITICAL(&stateMux);if(nextFrame)frameAge=now-frameStamps[nextFrame%8].captured;portEXIT_CRITICAL(&stateMux);
  char json[1536];
  snprintf(json,sizeof(json),
    "{\"version\":\"10.6\",\"uptimeMs\":%lu,\"bootId\":%lu,\"resetReason\":%d,\"clients\":%u,\"rssi\":%s,"
    "\"channel\":%d,\"channelScores\":[%ld,%ld,%ld],\"apConfigError\":%d,\"txQuarterDbm\":%d,"
    "\"wifiDisconnects\":%lu,\"lastWifiReason\":%lu,\"controlClosed\":%lu,"
    "\"cameraSent\":%lu,\"cameraWaits\":%lu,\"cameraSendErrors\":%lu,\"controlSendErrors\":%lu,"
    "\"cameraSendLastMs\":%lu,\"cameraSendMaxMs\":%lu,\"controlSendLastMs\":%lu,"
    "\"cameraDriverBuffers\":%lu,\"captureAgeAtReadMs\":%lu,\"cameraAgeAtSendMs\":%lu,\"captureFailures\":%lu,\"captureDrops\":%lu,\"captureLastMs\":%lu,\"lastJpegBytes\":%lu,\"cachedFrameAgeMs\":%lu,"
    "\"freeInternalHeap\":%u,\"minInternalHeap\":%u,\"largestInternalBlock\":%u}",
    (unsigned long)now,(unsigned long)bootId,int(esp_reset_reason()),unsigned(WiFi.softAPgetStationNum()),signal,
    selectedChannel,(long)channelScores[0],(long)channelScores[1],(long)channelScores[2],apConfigError.load(),apActualPower.load(),
    (unsigned long)wifiDisconnects.load(),(unsigned long)apLastReason.load(),(unsigned long)controlClosed.load(),
    (unsigned long)cameraSent.load(),(unsigned long)cameraWaits.load(),(unsigned long)cameraSendErrors.load(),(unsigned long)controlSendErrors.load(),
    (unsigned long)cameraSendLastMs.load(),(unsigned long)cameraSendMaxMs.load(),(unsigned long)controlSendLastMs.load(),
    (unsigned long)cameraDriverBuffers.load(),(unsigned long)captureAgeAtReadMs.load(),(unsigned long)cameraAgeAtSendMs.load(),
    (unsigned long)captureFailures.load(),(unsigned long)captureDrops.load(),(unsigned long)captureLastMs.load(),(unsigned long)cameraLastBytes.load(),(unsigned long)frameAge,
    unsigned(heap_caps_get_free_size(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT)),unsigned(heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT)),
    unsigned(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT)));
  httpd_resp_set_type(req,"application/json");httpd_resp_set_hdr(req,"Cache-Control","no-store");httpd_resp_set_hdr(req,"Connection","close");
  return closeHttpResponse(req,httpd_resp_send(req,json,HTTPD_RESP_USE_STRLEN));
}
esp_err_t diagnosticHandler(httpd_req_t *req){
  httpd_resp_set_hdr(req,"Connection","close");
  httpd_resp_set_type(req,"text/plain; charset=utf-8");httpd_resp_set_hdr(req,"Cache-Control","no-store");
  return closeHttpResponse(req,httpd_resp_sendstr(req,savedCrash));
}

bool addRoute(httpd_handle_t server,const char *uri,httpd_method_t method,esp_err_t(*handler)(httpd_req_t*),bool ws=false){
  httpd_uri_t route={};route.uri=uri;route.method=method;route.handler=handler;route.is_websocket=ws;
  return httpd_register_uri_handler(server,&route)==ESP_OK;
}
void initServers(){
  httpd_config_t config=HTTPD_DEFAULT_CONFIG();
  config.server_port=80;config.ctrl_port=32768;config.stack_size=8192;
  config.max_open_sockets=4;
  config.core_id=1;config.task_priority=3;
  config.keep_alive_enable=true;config.keep_alive_idle=3;config.keep_alive_interval=1;config.keep_alive_count=3;
  config.recv_wait_timeout=1;config.send_wait_timeout=1;config.lru_purge_enable=false; // A new auxiliary socket must not evict the motor owner.
  if(httpd_start(&webServer,&config)==ESP_OK){
    const bool ok=addRoute(webServer,"/",HTTP_GET,pageHandler)&&addRoute(webServer,"/status",HTTP_GET,statusHandler)&&
      addRoute(webServer,"/stop",HTTP_POST,stopHandler)&&addRoute(webServer,"/ws",HTTP_GET,wsHandler,true)&&
      addRoute(webServer,"/diagnostico",HTTP_GET,diagnosticHandler)&&addRoute(webServer,"/rede",HTTP_GET,networkDiagnosticHandler);
    if(!ok){httpd_stop(webServer);webServer=nullptr;}
  }
  config.server_port=81;config.ctrl_port=32769;config.stack_size=6144;config.task_priority=2;
  config.max_open_sockets=2; // TCP keepalive reclaims broken peers; no eviction of a live stream.
  if(httpd_start(&frameServer,&config)==ESP_OK){
    if(!addRoute(frameServer,"/frame",HTTP_GET,frameHandler)||!addRoute(frameServer,"/camera",HTTP_GET,cameraWsHandler,true)){
      httpd_stop(frameServer);frameServer=nullptr;
    }
  }
}

int chooseAccessPointChannel(){
  if(AP_CHANNEL)return AP_CHANNEL;
  const int channels[3]={1,6,11};
  for(auto &score:channelScores)score=0;
  // Survey only at boot, before camera/server tasks and before any movement.
  if(!WiFi.mode(WIFI_STA))return 6;
  WiFi.setAutoReconnect(false);WiFi.disconnect(false,false);
  int count=WiFi.scanNetworks(true,true,false,120);
  const uint32_t started=millis();
  while(count==WIFI_SCAN_RUNNING&&static_cast<uint32_t>(millis()-started)<4000){
    vTaskDelay(pdMS_TO_TICKS(20));count=WiFi.scanComplete();
  }
  if(count==WIFI_SCAN_RUNNING){esp_wifi_scan_stop();count=-2;}
  for(int i=0;i<count&&i<255;++i){
    const int channel=WiFi.channel(i),rssi=WiFi.RSSI(i);
    const int strength=rssi<-95?1:rssi> -30?66:rssi+96;
    for(int j=0;j<3;++j){const int distance=abs(channel-channels[j]);if(distance<5)channelScores[j]+=strength*(5-distance);}
  }
  WiFi.scanDelete();WiFi.mode(WIFI_OFF);
  int best=1;for(int j=0;j<3;++j)if(channelScores[j]<channelScores[best])best=j;
  return channels[best];
}
void networkEvent(WiFiEvent_t event,WiFiEventInfo_t info){
  if(event==ARDUINO_EVENT_WIFI_AP_STACONNECTED)apHasClient=true;
  if(event==ARDUINO_EVENT_WIFI_AP_STADISCONNECTED||event==ARDUINO_EVENT_WIFI_AP_STOP){
    apHasClient=false;if(event==ARDUINO_EVENT_WIFI_AP_STOP)apReady=false;
    if(event==ARDUINO_EVENT_WIFI_AP_STADISCONNECTED){++wifiDisconnects;apLastReason=info.wifi_ap_stadisconnected.reason;}
    portENTER_CRITICAL(&stateMux);drive.stop(DriveControl::WIFI_LOST);portEXIT_CRITICAL(&stateMux);
  }
}
bool startAccessPoint(){
  apReady=false;apHasClient=false;selectedChannel=chooseAccessPointChannel();
  WiFi.onEvent(networkEvent);
  if(!WiFi.mode(WIFI_AP))return false;
  const IPAddress ip(192,168,4,1),mask(255,255,255,0);
  if(!WiFi.softAPConfig(ip,ip,mask)||!WiFi.softAP(AP_SSID,AP_PASSWORD,selectedChannel,0,1)){
    WiFi.softAPdisconnect(true);return false;
  }
  const esp_err_t ps=esp_wifi_set_ps(WIFI_PS_NONE);
  const esp_err_t bw=esp_wifi_set_bandwidth(WIFI_IF_AP,WIFI_BW_HT20);
  const esp_err_t proto=esp_wifi_set_protocol(WIFI_IF_AP,WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N);
  const bool power=WiFi.setTxPower(AP_TX_POWER);int8_t actual=-1;
  if(esp_wifi_get_max_tx_power(&actual)==ESP_OK)apActualPower=actual;
  apConfigError=ps!=ESP_OK?ps:bw!=ESP_OK?bw:proto!=ESP_OK?proto:power?0:ESP_FAIL;
  apReady=true;
  Serial.printf("REDE: %s | canal=%d | pontuacao 1/6/11=%ld/%ld/%ld | configuracao=%d | TX=%d (0,25dBm)\n",
    AP_SSID,selectedChannel,(long)channelScores[0],(long)channelScores[1],(long)channelScores[2],apConfigError.load(),apActualPower.load());
  Serial.println("Conecte o notebook a rede do carrinho e abra http://192.168.4.1");
  return true;
}

void setup(){
  initActuators(); // Enables em zero antes da inicializacao da camera/rede.
  Serial.begin(115200);
  bootId=esp_random();if(!bootId)bootId=1;
  Serial.printf("\nSEGUIDOR V10.6 PULSOS COM CONFIRMACAO SEM SERVO | Reset=%d (brownout=%d) | Boot=%lu\n",int(esp_reset_reason()),int(ESP_RST_BROWNOUT),(unsigned long)bootId);
  readSavedCrash();
  pinMode(4,OUTPUT);digitalWrite(4,LOW);
  const bool accessPointStarted=startAccessPoint();
  cameraReady=initCamera();
  if(cameraReady&&!startCameraPipeline())cameraReady=false;
  Serial.printf("Camera: %s | PWM: %s\n",esp_err_to_name(cameraError),esp_err_to_name(actuatorError));
  if(accessPointStarted)initServers();
  else Serial.println("ERRO: rede propria indisponivel. Reinicie a placa.");
  if(!webServer||!frameServer){
    portENTER_CRITICAL(&stateMux);drive.stop(DriveControl::FAULT);portEXIT_CRITICAL(&stateMux);
    Serial.println("ERRO: inicializacao incompleta dos servidores; inicio bloqueado.");
  }
  // The browser uses WS on port 81; do not reserve a task/socket for the unused UDP bridge.
  Serial.println(webServer?"Painel preparado. Abra http://192.168.4.1 na rede do carrinho.":"ERRO: servidor indisponivel");
}
void loop(){
  // Events handle loss immediately; polling only reconciles station availability.
  apHasClient=apReady.load()&&WiFi.AP.started()&&WiFi.softAPgetStationNum()>0;
  vTaskDelay(pdMS_TO_TICKS(100));
}
