"""
ESP32-S3 Voice Assistant HTTP Server - STREAMING OPTIMIZED
Handles: Audio Upload -> STT -> Parallel(LLM + Emotion) -> TTS -> Return MP3 + Emotion Header
NOW WITH PROPER STREAMING TO PREVENT AUDIO CUTTING
"""

from flask import Flask, request, Response, jsonify
import wave
import io
from pydub import AudioSegment
import os
import json
import re
import time
from groq import Groq
from langchain_groq import ChatGroq
from gtts import gTTS
from customagent.agent import robot_graph
from langchain_core.messages import HumanMessage, ToolMessage
from typing import Literal, Generator
from pydantic import BaseModel
from langchain_core.runnables import chain, RunnableParallel

# ======================== Configuration ========================
GROQ_API_KEY = "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"
llm = ChatGroq(model="meta-llama/llama-4-maverick-17b-128e-instruct", api_key=GROQ_API_KEY, max_tokens=40)

SAMPLE_RATE = 8000
CHANNELS = 1
SAMPLE_WIDTH = 2

# Streaming configuration - OPTIMIZED for stable playback
CHUNK_SIZE = 4096  # Back to 4KB for consistent network flow
CHUNK_DELAY_MS = 20  # Moderate delay for stable streaming (not too fast, not too slow)

if not GROQ_API_KEY:
    print("❌ ERROR: GROQ_API_KEY not set!")
    exit(1)

groq_client = Groq(api_key=GROQ_API_KEY)
app = Flask(__name__)

class OneWordOutput(BaseModel):
    choice: Literal[
        "A0_Wakeup",
        "A1_Reset",
        "A2_Move_Right",
        "A3_Move_Left",
        "A4_Blink_Long",
        "A5_Blink_Short",
        "A6_Happy",
        "A7_Sleep",
        "A8_Random_Saccades",
        "M0_Stop_Automation",
        "M1_Demo_Mode",
        "M2_Idle_Mode",
        "M3_Attentive_Mode",
        "M4_Sleepy_Mode",
        "M5_Random_Mode",
        "M6_Custom_Sequence"]

@chain
def decide_emotion(user_query):
    """Decide emotion/mode based on user query"""
    llm_emotion = ChatGroq(model="meta-llama/llama-4-maverick-17b-128e-instruct", api_key=GROQ_API_KEY, max_tokens=40)
    llm_structured = llm_emotion.with_structured_output(OneWordOutput)
    decide_prompt = f"""
        Your job is to decide which agent option mode emotion to use based on the user task.
        you have 12 options:
        🔹 Actions (A0–A8)
        | Code   | Mode / Description                                      |
        | ------ | ------------------------------------------------------- |
        | **A0** | Wakeup – wakes the system or character up               |
        | **A1** | Reset – resets to default state                         |
        | **A2** | Move Right – moves eyes or object to the right          |
        | **A3** | Move Left – moves eyes or object to the left            |
        | **A4** | Blink Long – performs a long blink                      |
        | **A5** | Blink Short – performs a quick blink                    |
        | **A6** | Happy – displays or acts happy                          |
        | **A7** | Sleep – enters sleep state                              |
        ⚙️ Automation Modes (M0–M6)
        | Code   | Mode / Description                                  |
        | ------ | --------------------------------------------------- |
        | **M0** | Stop Automation – stops any automated behavior      |
        | **M1** | Demo Mode – cycles through all actions              |
        | **M2** | Idle Mode – performs natural idle movements         |
        | **M3** | Attentive Mode – alert and focused behavior         |
        | **M4** | Sleepy Mode – slow, tired movement pattern          |
        """
    decide_message = [
            {"role": "system", "content": decide_prompt},
            {"role": "user", "content": user_query}
            ]

    try:
        response = llm_structured.invoke(decide_message)
        return response.choice
    except Exception as e:
        # Structured output / function-calling failed for this model call. Log and fallback.
        print(f"   ⚠️  Structured output failed: {e}")

        # Fallback: call the same model without structured output and try to map the plain text
        try:
            plain = llm_emotion.invoke(decide_message)
            # plain may be a complex object; coerce to string
            plain_text = ''
            try:
                # If it's a list/dict-like response, try common attributes
                if hasattr(plain, 'content'):
                    plain_text = str(plain.content)
                else:
                    plain_text = str(plain)
            except Exception:
                plain_text = str(plain)

            plain_text_low = plain_text.lower()

            # exact code mapping if model returned e.g. "A0_Wakeup" or "A0"
            code_match = re.search(r"\b([AM][0-9])(?:_[A-Za-z0-9_]+)?\b", plain_text)
            if code_match:
                short_code = code_match.group(1)
            else:
                short_code = None

            # mapping from short codes to allowed OneWordOutput choices
            mapping = {
                'A0': 'A0_Wakeup',
                'A1': 'A1_Reset',
                'A2': 'A2_Move_Right',
                'A3': 'A3_Move_Left',
                'A4': 'A4_Blink_Long',
                'A5': 'A5_Blink_Short',
                'A6': 'A6_Happy',
                'A7': 'A7_Sleep',
                'A8': 'A8_Random_Saccades',
                'M0': 'M0_Stop_Automation',
                'M1': 'M1_Demo_Mode',
                'M2': 'M2_Idle_Mode',
                'M3': 'M3_Attentive_Mode',
                'M4': 'M4_Sleepy_Mode',
                'M5': 'M5_Random_Mode',
                'M6': 'M6_Custom_Sequence'
            }

            if short_code and short_code in mapping:
                return mapping[short_code]

            # keyword heuristics fallback
            heur = {
                'wakeup': 'A0_Wakeup',
                'reset': 'A1_Reset',
                'right': 'A2_Move_Right',
                'left': 'A3_Move_Left',
                'long blink': 'A4_Blink_Long',
                'blink long': 'A4_Blink_Long',
                'short blink': 'A5_Blink_Short',
                'blink short': 'A5_Blink_Short',
                'happy': 'A6_Happy',
                'sleep': 'A7_Sleep',
                'saccade': 'A8_Random_Saccades',
                'random saccades': 'A8_Random_Saccades',
                'stop': 'M0_Stop_Automation',
                'demo': 'M1_Demo_Mode',
                'idle': 'M2_Idle_Mode',
                'attentive': 'M3_Attentive_Mode',
                'sleepy': 'M4_Sleepy_Mode',
                'random mode': 'M5_Random_Mode',
                'custom': 'M6_Custom_Sequence'
            }

            for k, v in heur.items():
                if k in plain_text_low:
                    return v

            # As a final fallback, return reset
            return 'A1_Reset'

        except Exception as e2:
            print(f"   ⚠️  Fallback plain-text call failed: {e2}")
            return 'A1_Reset'

@chain
def generate_llm_response(transcription):
    """Generate LLM response from transcription"""
    result = robot_graph.invoke(
        {"messages": [HumanMessage(content=transcription)]}, 
        config={"configurable": {"thread_id": "main_thread"}}
    )
    return result

def create_parallel_chain():
    """Create parallel processing chain"""
    parallel_chain = RunnableParallel(
        emotion=decide_emotion,
        llm_response=generate_llm_response
    )
    return parallel_chain

def process_llm_response(response: str) -> tuple[str, dict]:
    """Process LLM response"""
    try:
        response_clean = response.strip()
        
        if response_clean.startswith('{') and response_clean.endswith('}'):
            command = json.loads(response_clean)
            
            if "direction" in command and "speed" in command:
                print(f"   🤖 Detected movement command: {command}")
                text_to_speak = "I am coming sir"
                return text_to_speak, command
        
        return response, None
        
    except json.JSONDecodeError:
        return response, None
    except Exception as e:
        print(f"   ⚠️  Error processing response: {e}")
        return response, None

def create_wav_buffer(audio_data: bytes) -> bytes:
    """Convert raw PCM to WAV"""
    wav_buffer = io.BytesIO()
    with wave.open(wav_buffer, 'wb') as wav_file:
        wav_file.setnchannels(CHANNELS)
        wav_file.setsampwidth(SAMPLE_WIDTH)
        wav_file.setframerate(SAMPLE_RATE)
        wav_file.writeframes(audio_data)
    wav_buffer.seek(0)
    return wav_buffer.getvalue()

def transcribe_audio(audio_data: bytes) -> str:
    """Transcribe audio using Groq"""
    try:
        wav_data = create_wav_buffer(audio_data)
        wav_file = io.BytesIO(wav_data)
        wav_file.name = "audio.wav"
        
        print("   ⏳ Sending to Groq API...")
        transcription = groq_client.audio.transcriptions.create(
            file=wav_file,
            model="whisper-large-v3-turbo",
            response_format="text",
            language="en",
            temperature=0.0
        )
        
        return transcription.strip()
        
    except Exception as e:
        print(f"   ❌ Transcription error: {e}")
        return f"[Error: {str(e)}]"

def generate_tts_pcm(text: str, lang: str = "en") -> bytes:
    """Generate TTS audio as 22.05kHz 16-bit mono RAW PCM (no WAV header) - OPTIMIZED"""
    try:
        start_time = time.time()
        
        # Generate MP3 with gTTS
        print(f"   🎙️ Generating TTS audio...")
        tts = gTTS(text=text, lang=lang, slow=False)
        mp3_buffer = io.BytesIO()
        tts.write_to_fp(mp3_buffer)
        mp3_buffer.seek(0)
        
        gen_time = time.time() - start_time
        print(f"   ⏱️  TTS generation took: {gen_time:.2f}s")
        
        # Convert MP3 to correct format
        conv_start = time.time()
        audio = AudioSegment.from_mp3(mp3_buffer)
        audio = audio.set_frame_rate(22050)  # Match ESP32
        audio = audio.set_channels(1)        # Mono
        audio = audio.set_sample_width(2)    # 16-bit
        
        # Export as RAW PCM (no header)
        raw_buffer = io.BytesIO()
        audio.export(raw_buffer, format="s16le")  # signed 16-bit little-endian
        raw_buffer.seek(0)
        pcm_data = raw_buffer.read()
        
        conv_time = time.time() - conv_start
        total_time = time.time() - start_time
        print(f"   ⏱️  Conversion took: {conv_time:.2f}s | Total: {total_time:.2f}s")
        print(f"   📊 Generated PCM size: {len(pcm_data):,} bytes (22.05kHz, 16-bit, mono)")
        return pcm_data
        
    except Exception as e:
        print(f"   ❌ TTS error: {e}")
        import traceback
        traceback.print_exc()
        return None

def stream_audio_chunks(audio_data: bytes) -> Generator[bytes, None, None]:
    """
    Stream audio data in controlled chunks to prevent flooding ESP32.
    This prevents the audio from being cut off or distorted.
    """
    total_size = len(audio_data)
    chunks_sent = 0
    
    print(f"   🔄 Starting audio stream: {total_size} bytes in {CHUNK_SIZE} byte chunks")
    
    for offset in range(0, total_size, CHUNK_SIZE):
        chunk = audio_data[offset:offset + CHUNK_SIZE]
        chunks_sent += 1
        
        # Send chunk
        yield chunk
        
        # Calculate and log progress
        progress_pct = (offset + len(chunk)) * 100 // total_size
        if chunks_sent % 5 == 0:  # Log every 5 chunks to avoid spam
            print(f"   📤 Stream progress: {progress_pct}% ({chunks_sent} chunks sent)")
        
        # Small delay between chunks to allow ESP32 to process
        # This is CRITICAL to prevent buffer overflow and audio cutting
        time.sleep(CHUNK_DELAY_MS / 1000.0)
    
    print(f"   ✅ Audio stream complete: {chunks_sent} chunks sent")

# ======================== OPTIMIZED STREAMING FUNCTION ========================
import threading
from queue import Queue

class OptimizedAudioStreamer:
    """Handles fast streaming with early audio start"""
    def __init__(self):
        self.tts_queue = Queue(maxsize=200)  # Larger queue for buffering
        self.emotion_code = "A1_Reset"
        self.streaming_started = False
        self.total_queued = 0
    
    def stream_chunks(self):
        """Generator for streaming chunks as they become available"""
        chunks_sent = 0
        total_size = 0
        first_chunk_time = None
        
        # Send chunks as they arrive from TTS generation
        while True:
            try:
                chunk = self.tts_queue.get(timeout=30)  # 30s timeout
                if chunk is None:  # End of stream marker
                    print(f"   ✅ Audio stream complete: {chunks_sent} chunks, {total_size:,} bytes")
                    break
                
                if first_chunk_time is None:
                    first_chunk_time = time.time()
                    print(f"   🚀 FIRST CHUNK SENT! Starting playback on ESP32...")
                
                yield chunk
                chunks_sent += 1
                total_size += len(chunk)
                
                if chunks_sent % 20 == 0:  # Log every 20 chunks to reduce spam
                    print(f"   📤 Stream progress: {chunks_sent} chunks sent ({total_size:,} bytes)")
                
                # Reduced delay for faster streaming
                time.sleep(CHUNK_DELAY_MS / 1000.0)
            except Exception as e:
                print(f"   ⚠️ Stream error: {e}")
                break
    
    def queue_tts_chunks(self, tts_audio):
        """Queue audio chunks from TTS (runs in separate thread)"""
        try:
            chunks_queued = 0
            for offset in range(0, len(tts_audio), CHUNK_SIZE):
                chunk = tts_audio[offset:offset + CHUNK_SIZE]
                self.tts_queue.put(chunk)
                chunks_queued += 1
                self.total_queued += len(chunk)
            
            print(f"   📊 Queued {chunks_queued} chunks ({self.total_queued:,} bytes total)")
            self.tts_queue.put(None)  # End of stream marker
        except Exception as e:
            print(f"   ❌ Error queuing TTS: {e}")
            self.tts_queue.put(None)

# ======================== HTTP Endpoint ========================
@app.route('/upload', methods=['POST'])
def upload_audio():
    print("\n" + "="*60)
    print("📥 Received audio upload request")
    print("="*60)
    
    start_total = time.time()
    
    try:
        # Get audio data
        audio_data = request.data
        audio_size = len(audio_data)
        duration = audio_size / (SAMPLE_RATE * SAMPLE_WIDTH)
        
        print(f"📥 Audio size: {audio_size:,} bytes ({duration:.1f}s)")
        
        # Step 1: Transcribe (this happens first, takes time anyway)
        transcribe_start = time.time()
        transcription = transcribe_audio(audio_data)
        transcribe_time = time.time() - transcribe_start
        print(f"✅ Transcription: \"{transcription}\" ({transcribe_time:.2f}s)")
        
        if transcription and "[Error" not in transcription:
            try:
                # Create parallel chain
                parallel_chain = create_parallel_chain()
                
                # Step 2: Start parallel processing (LLM + Emotion)
                print(f"⚡ Running parallel processing (LLM + Emotion)...")
                parallel_start = time.time()
                parallel_result = parallel_chain.invoke(transcription)
                parallel_time = time.time() - parallel_start
                
                emotion_code = parallel_result["emotion"]
                llm_result = parallel_result["llm_response"]
                
                print(f"✅😊 Emotion: {emotion_code} ({parallel_time:.2f}s)")
                
                # Process LLM result
                last_message = llm_result["messages"][-1]
                
                if isinstance(last_message, ToolMessage):
                    response_raw = last_message.content
                    print(f"✅🤖 Tool Result: \"{response_raw}\"")
                    text_to_speak, command = process_llm_response(response_raw)
                else:
                    response_raw = last_message.content if hasattr(last_message, 'content') else str(last_message)
                    print(f"✅🤖 Response: \"{response_raw}\"")
                    text_to_speak, command = response_raw, None
                
            except Exception as e:
                print(f"⚠️ LLM/Emotion processing error: {e}")
                import traceback
                traceback.print_exc()
                text_to_speak = "I had trouble processing that request."
                emotion_code = "A1_Reset"
                command = None
            
            if text_to_speak and text_to_speak.strip():
                # Step 3: Generate TTS AND START STREAMING AT SAME TIME
                try:
                    # Create streamer
                    streamer = OptimizedAudioStreamer()
                    streamer.emotion_code = emotion_code
                    
                    # Start TTS generation in background thread
                    tts_start = time.time()
                    print(f"🎙️  Starting TTS generation in background...")
                    
                    def tts_worker():
                        tts_audio = generate_tts_pcm(text_to_speak)
                        if tts_audio:
                            streamer.queue_tts_chunks(tts_audio)
                        else:
                            streamer.tts_queue.put(None)
                    
                    tts_thread = threading.Thread(target=tts_worker, daemon=False)
                    tts_thread.start()
                    
                    # Create streaming response IMMEDIATELY
                    # No waiting for full TTS to complete!
                    response = Response(streamer.stream_chunks(), mimetype='application/octet-stream')
                    response.headers['X-Emotion-Code'] = emotion_code
                    response.headers['Cache-Control'] = 'no-cache'
                    response.headers['Connection'] = 'keep-alive'
                    response.headers['Transfer-Encoding'] = 'chunked'
                    
                    if command:
                        response.headers['X-Command'] = json.dumps(command)
                    
                    total_time = time.time() - start_total
                    print(f"✅ Streaming response prepared in {total_time:.2f}s - Starting stream NOW!")
                    print(f"⏱️  Breakdown: Transcribe={transcribe_time:.2f}s, Parallel={parallel_time:.2f}s, Setup={total_time-transcribe_time-parallel_time:.2f}s")
                    
                    return response
                        
                except Exception as e:
                    print(f"⚠️ TTS generation error: {e}")
                    import traceback
                    traceback.print_exc()
                    # Return simple error audio
                    error_audio = generate_tts_pcm("TTS failed")
                    response = Response(error_audio, mimetype='application/octet-stream')
                    response.headers['X-Emotion-Code'] = "A1_Reset"
                    return response
            else:
                print("⚠️ No text to speak")
                error_audio = generate_tts_pcm("I couldn't generate a response.")
                response = Response(error_audio, mimetype='application/octet-stream')
                response.headers['X-Emotion-Code'] = "A1_Reset"
                return response
        else:
            print("⚠️ Transcription failed or empty")
            error_audio = generate_tts_pcm("No speech detected")
            response = Response(error_audio, mimetype='application/octet-stream')
            response.headers['X-Emotion-Code'] = "A1_Reset"
            return response
            
    except Exception as e:
        print(f"❌ Critical error: {e}")
        import traceback
        traceback.print_exc()
        
        # Always return something valid
        try:
            error_audio = generate_tts_pcm("Sorry, there was an error.")
            response = Response(error_audio, mimetype='application/octet-stream')
            response.headers['X-Emotion-Code'] = "A1_Reset"
            return response
        except:
            # Last resort: return empty response with header
            return Response(b'', mimetype='application/octet-stream', 
                          headers={'X-Emotion-Code': 'A1_Reset'})
        
@app.route('/health', methods=['GET'])
def health():
    """Health check endpoint"""
    return jsonify({"status": "ok", "service": "voice_assistant", "streaming": True})

if __name__ == "__main__":
    print("╔══════════════════════════════════════════════════════════╗")
    print("║                                                          ║")
    print("║   🎙️  Voice Assistant HTTP Server (STREAMING) 🔊       ║")
    print("║   STT -> Parallel(LLM + Emotion) -> TTS -> Stream+Header║")
    print("║   ✅ PROPER CHUNKED STREAMING TO PREVENT AUDIO CUTTING  ║")
    print("║   ⚡ Parallel Processing: LLM & Emotion                 ║")
    print("║                                                          ║")
    print("╚══════════════════════════════════════════════════════════╝")
    print(f"⚙️  Configuration:")
    print(f"   Server: http://0.0.0.0:5000")
    print(f"   Sample Rate: {SAMPLE_RATE} Hz (input), 22.05 kHz (output)")
    print(f"   Format: {SAMPLE_WIDTH*8}-bit PCM Mono")
    print(f"   Chunk Size: {CHUNK_SIZE} bytes")
    print(f"   Chunk Delay: {CHUNK_DELAY_MS}ms (prevents flooding ESP32)")
    print(f"   STT Model: whisper-large-v3-turbo")
    print(f"   LLM Model: llama-4-maverick-17b-128e-instruct")
    print(f"   TTS: gTTS")
    print(f"   Response: Streamed PCM + X-Emotion-Code header")
    print(f"🚀 Server starting...")
    print("="*60 + "\n")
    
    app.run(host='192.168.1.3', port=5000, debug=False, threaded=True)
