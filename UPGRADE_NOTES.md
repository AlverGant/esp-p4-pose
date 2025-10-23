# Upgrade Notes - v1.1.0

## Mudanças Implementadas

### 1. Upgrade para ESP-DL Nova API

#### ❌ **Arquivos Removidos (agora usa componentes ESP-DL oficiais)**
- `main/coco_pose.cpp` → Movido para `coco_pose.cpp.bak`
- `main/coco_pose.hpp` → Movido para `coco_pose.hpp.bak`

**Motivo**: A nova versão do ESP-DL implementa o componente `coco_pose` com uma API melhorada que inclui:
- Sistema de `lazy_load` para carregamento diferido de modelos
- Suporte nativo para múltiplos modelos (V1 e V2)
- Thresholds configuráveis por construtor

#### ✅ **Arquivos Modificados**

**main/CMakeLists.txt**
```diff
- "coco_pose.cpp"
+ REQUIRES
+   coco_pose
```
- Removido `coco_pose.cpp` dos SRCS
- Adicionado `coco_pose` como dependência de componente

**main/idf_component.yml**
```diff
- espressif/coco_pose:
-   version: "^0.2.0"
-   override_path: "/path/to/esp-dl/models/coco_pose"
+ espressif/coco_pose:
+   path: "/path/to/esp-dl/models/coco_pose"
```
- Removido `version` e `override_path`
- Usado apenas `path` para componente local

**main/pose_overlay.cpp**
```diff
- s_pose = new COCOPose(COCOPose::YOLO11N_POSE_S8_V2);
+ s_pose = new COCOPose(COCOPose::YOLO11N_POSE_S8_V2, false);
```
- Adicionado parâmetro `lazy_load = false` para carregamento imediato
- Atualizado comentário com informações do modelo V2

**main/app_main.c**
```diff
- #define POSE_INPUT_RES 960
+ #define POSE_INPUT_RES 640
```
- Reduzido resolução de inferência de 960x960 para 640x640
- Atualizados comentários com benchmarks de performance

---

## Novo Fluxo de Compilação

### Antes (API Antiga)
```
esp-p4-pose/
├── main/
│   ├── coco_pose.cpp       ← Wrapper customizado
│   ├── coco_pose.hpp       ← Header customizado
│   └── pose_overlay.cpp    ← Usa wrapper local
```

### Depois (Nova API)
```
esp-p4-pose/
├── main/
│   └── pose_overlay.cpp    ← Usa componente ESP-DL diretamente
└── (depende de)
    esp-dl/models/coco_pose/
    ├── coco_pose.cpp       ← Implementação oficial
    ├── coco_pose.hpp       ← API oficial
    └── idf_component.yml
```

---

## Configuração do Modelo V2

### Verificar no `idf.py menuconfig`:

```
Component config → COCO Pose
  [X] FLASH_COCO_POSE_YOLO11N_POSE_S8_V2
  [ ] FLASH_COCO_POSE_YOLO11N_POSE_S8_V1

  Default Model → YOLO11N_POSE_S8_V2

  Model Location → FLASH rodata
```

### Verificar Arquivo do Modelo:

```bash
ls -lh /home/alvaro/Downloads/esp-dl/models/coco_pose/models/p4/
# Deve mostrar:
# coco_pose_yolo11n_pose_s8_v2.espdl (3.0M)
```

---

## Performance Esperada

| Métrica | Antes (V1 @ 960) | Depois (V2 @ 640) | Ganho |
|---------|------------------|-------------------|-------|
| **Inferência** | ~6.0s | ~3.0s | **2x** ⚡ |
| **mAP50-95** | 0.431 | 0.449 | **+4.2%** 📈 |
| **Memória** | 1.77MB | 0.78MB | **-56%** 💾 |
| **FPS equiv** | 0.17 | 0.33 | **2x** 🎥 |

---

## Troubleshooting

### ❌ Erro: "missing and no known rule to make it"
```
ninja: error: 'coco_pose.cpp', needed by '...', missing
```
**Solução**: Remova `coco_pose.cpp` do `CMakeLists.txt` e adicione `coco_pose` em REQUIRES.

### ❌ Erro: "invalid new-expression of abstract class type"
```
error: invalid new-expression of abstract class type 'COCOPose'
note: 'virtual void load_model()' is pure
```
**Solução**: Use o componente oficial do ESP-DL (não crie wrapper local).

### ❌ Erro: "doesn't match any versions"
```
ERROR: Because project depends on espressif/coco_pose (^0.2.0)
```
**Solução**: Use `path:` ao invés de `version:` + `override_path:` no idf_component.yml.

### ✅ Verificar Compilação Bem-Sucedida

Procure na saída do build:
```
[1820/1828] Building CXX .../coco_pose.cpp.obj
[1821/1828] Linking C static library .../libcoco_pose.a
...
Project build complete.
```

### ✅ Verificar Logs de Runtime

```
I (xxx) pose: Pose initialized (YOLO11n-Pose V2 with QAT, loaded immediately)
I (xxx) app_main: Submitting pose buffer 640x640
I (xxx) POSE: inference done in 3.15s, persons=2
```

Confirme:
- ✅ "V2 with QAT" no log de inicialização
- ✅ "640x640" no log de submissão
- ✅ ~3s de tempo de inferência

---

## Rollback (Se Necessário)

Para reverter para a versão antiga:

```bash
cd /home/alvaro/Downloads/esp-p4-pose

# 1. Restaurar arquivos customizados
mv main/coco_pose.cpp.bak main/coco_pose.cpp
mv main/coco_pose.hpp.bak main/coco_pose.hpp

# 2. Reverter CMakeLists.txt
git checkout main/CMakeLists.txt

# 3. Reverter resolução
# Editar main/app_main.c: POSE_INPUT_RES 960

# 4. Reverter pose_overlay.cpp
# Editar: new COCOPose() (sem parâmetros)

# 5. Rebuild
idf.py fullclean && idf.py build
```

---

## Próximos Passos Sugeridos

### Validação
1. Flash e teste no hardware real
2. Meça tempo de inferência (~3s esperado)
3. Valide detecção de quedas em cenários reais
4. Compare taxa de falsos positivos/negativos

### Otimizações Futuras (Roadmap v1.2)
- [ ] Adicionar YOLO11n-320 para contexto de cena
- [ ] Implementar ESPDet-Pico como pré-filtro
- [ ] Tracking temporal com LSTM
- [ ] Multi-threading para detecção paralela

---

**Data**: 2025-01-23
**Versão**: v1.1.0
**ESP-DL**: v3.2.0 (coco_pose 0.3.0)
**ESP-IDF**: v5.5.1
