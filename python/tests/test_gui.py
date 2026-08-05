"""The GUI's job rendering, exercised for real.

`gui.html` has no build step and no test runner, and `dispatch()` deliberately
swallows exceptions ("partial logs / dropped cycles are normal") — which means a
broken module fails *silently*, showing an empty panel rather than an error.
That makes some check on the JavaScript worth having.

This extracts the module under test out of the single inline `<script>`, runs it
in QuickJS against stub DOM objects, and drives it with the exact message
sequence the host emits. It needs no browser and no npm. If no JS engine is
present the test skips rather than pretending to pass.
"""

from __future__ import annotations

import re
import shutil
import subprocess
import textwrap
from pathlib import Path

import pytest

GUI = Path(__file__).resolve().parents[1] / 'gui' / 'gui.html'
QJS = shutil.which('qjs') or shutil.which('node')

pytestmark = pytest.mark.skipif(
    QJS is None, reason="no JS engine (qjs/node) available to parse gui.html")


def _script() -> str:
    """The GUI's inline JavaScript."""
    blocks = re.findall(r'<script>(.*?)</script>', GUI.read_text(), re.S)
    assert blocks, "gui.html has no inline script block"
    return '\n'.join(blocks)


def _run_js(source: str, tmp_path: Path) -> str:
    path = tmp_path / 'check.js'
    path.write_text(source)
    proc = subprocess.run([QJS, str(path)], capture_output=True, text=True, timeout=60)
    if proc.returncode != 0:
        raise AssertionError(f"JS failed:\n{proc.stdout}\n{proc.stderr}")
    return proc.stdout


def test_gui_script_parses(tmp_path):
    """A syntax error anywhere in gui.html breaks the entire GUI, silently."""
    js = _script().replace('\\', '\\\\').replace('`', '\\`').replace('${', '\\${')
    _run_js(f"try {{ new Function(`{js}`); }} catch (e) {{ "
            f"print('PARSE ERROR: ' + e); std_exit(1); }}\n"
            f"function std_exit(c){{ throw new Error('parse failed'); }}\n"
            f"print('ok');", tmp_path)


# Stub DOM: only what the Jobs module actually touches.
HARNESS = textwrap.dedent("""
    var _els = {};
    function _el(id){ if(!_els[id]) _els[id]={id:id,textContent:'',innerHTML:'',style:{}};
                      return _els[id]; }
    var document = { getElementById:_el };
    var _sent = [];
    var DataManager = { send:function(o){ _sent.push(o); return true; } };
    var _logs = [];
    function logMsg(m){ _logs.push(m); }
""")


def _jobs_module() -> str:
    js = _script()
    return js[js.index('const Jobs = {'):js.index('const Survey = {')]


def test_jobs_module_renders_a_full_lifecycle(tmp_path):
    body = """
        Jobs.on({type:'job_kinds', kinds:[{kind:'selftest',title:'Self test'}]});
        if (Jobs.kinds.length !== 1) throw new Error('job_kinds not stored');

        Jobs.on({type:'job_started', job_id:'j1', kind:'selftest',
                 title:'Self test', cancellable:true, started_ts:1});
        Jobs.on({type:'job_progress', job_id:'j1', frac:0.5, msg:'step 2/4'});
        Jobs.on({type:'job_log', job_id:'j1', msg:'halfway'});

        var html = _el('jobs-list').innerHTML;
        if (html.indexOf('50%') < 0)     throw new Error('no progress bar: '+html);
        if (html.indexOf('cancel') < 0)  throw new Error('no cancel affordance');
        if (_el('jobs-idle').textContent.indexOf('1 running') < 0)
            throw new Error('running count wrong');

        Jobs.on({type:'job_done', job_id:'j1', elapsed_s:3.2, result:{}, artifacts:[]});
        if (Jobs.jobs.get('j1').state !== 'done') throw new Error('done not recorded');
        if (_el('jobs-idle').textContent.indexOf('idle') < 0)
            throw new Error('should be idle again');
        print('ok');
    """
    assert 'ok' in _run_js(HARNESS + _jobs_module() + body, tmp_path)


def test_jobs_module_surfaces_rejections(tmp_path):
    """A rejected job never gets `job_started`; it must still be visible."""
    body = """
        Jobs.on({type:'job_failed', job_id:'job-rejected', kind:'nope',
                 elapsed_s:0, error:'unknown job kind: nope'});
        Jobs.onError({type:'error', code:'unknown_type', got:'bogus'});
        if (_logs.filter(function(l){return l.indexOf('unknown_type')>=0;}).length !== 1)
            throw new Error('unknown message type was not surfaced');
        Jobs.cancel('j2');
        if (_sent[_sent.length-1].type !== 'job_cancel')
            throw new Error('cancel not sent');
        print('ok');
    """
    assert 'ok' in _run_js(HARNESS + _jobs_module() + body, tmp_path)


def test_jobs_module_escapes_job_titles(tmp_path):
    """Job titles reach innerHTML; a job kind is not a trusted string."""
    body = """
        Jobs.on({type:'job_started', job_id:'jx', kind:'k',
                 title:'<img src=x onerror=alert(1)>'});
        if (_el('jobs-list').innerHTML.indexOf('<img') >= 0)
            throw new Error('job title was not escaped');
        print('ok');
    """
    assert 'ok' in _run_js(HARNESS + _jobs_module() + body, tmp_path)


def test_dispatch_routes_the_job_family():
    """The one line in `dispatch()` that makes every job message render."""
    js = _script()
    assert "t.startsWith('job_')" in js and 'Jobs.on(d)' in js
    assert "t === 'error'" in js and 'Jobs.onError(d)' in js
    assert "t.startsWith('fs_')" in js and 'Lab.on(d)' in js


def test_lab_mode_is_wired_into_the_mode_machinery():
    js = _script()
    assert "'operate','tune','diag','cal','lab'" in js, "setMode does not know LAB"
    html = GUI.read_text()
    assert "setMode('lab')" in html and 'body.mode-lab' in html
    assert '.for-lab' in html


# ── LAB: the generated tool runner ────────────────────────────────────────────

LAB_HARNESS = textwrap.dedent("""
    var _els={}, _created=[];
    function _mk(tag){ var e={tagName:tag,className:'',textContent:'',innerHTML:'',
      title:'',style:{},children:[],_on:{},
      appendChild:function(c){this.children.push(c);},
      addEventListener:function(ev,fn){this._on[ev]=fn;}}; _created.push(e); return e; }
    function _el(id){ if(!_els[id]) _els[id]=_mk('div'); _els[id].id=id; return _els[id]; }
    var document={getElementById:_el, createElement:_mk};
    var _sent=[]; var DataManager={send:function(o){_sent.push(o); return true;}};
    var _logs=[]; function logMsg(m){_logs.push(m);}
""")

LAB_SETUP = """
    Jobs.on({type:'job_kinds', kinds:[
      {kind:'replay_rtls', title:'Replay a session log', mode:'lab', args_schema:{
         log:{t:'path',kind:'logs',required:true},
         tagZ:{t:'number',default:0.22},
         gapWeights:{t:'bool',default:true},
         est:{t:'enum',choices:['ekf','mhe'],default:'ekf'}}},
      {kind:'selftest', title:'Self test', mode:'lab', args_schema:{}},
      {kind:'live_thing', title:'Not a lab tool', mode:'run', args_schema:{}}]});
    Lab.on({type:'fs_listing', kind:'logs', files:[
      {path:'matlab/logs/a.jsonl', name:'a.jsonl', bytes:1, mtime:2}]});
"""


def _lab_modules() -> str:
    """Jobs + the shared form generator + CalTools + Lab, as one snippet."""
    js = _script()
    return js[js.index('const Jobs = {'):js.index('const Survey = {')]


def _run_lab(body: str, tmp_path: Path) -> str:
    return _run_js(LAB_HARNESS + _lab_modules() + LAB_SETUP + body, tmp_path)


def test_lab_generates_its_form_from_the_advertised_schema(tmp_path):
    """A new server-side tool must cost zero markup here."""
    body = """
        if (Lab.kind !== 'replay_rtls') throw new Error('did not auto-select a lab tool');
        if (_el('lab-kind').innerHTML.indexOf('live_thing') >= 0)
            throw new Error('a non-lab tool leaked into the picker');
        var html = _el('lab-args').innerHTML;
        ['log','tagZ','gapWeights','est'].forEach(function(n){
            if (html.indexOf('arg-'+n) < 0) throw new Error('missing field '+n); });
        if (html.indexOf('checked') < 0) throw new Error('bool default not applied');
        if (html.indexOf('a.jsonl') < 0) throw new Error('file list not offered');
        print('ok');
    """
    assert 'ok' in _run_lab(body, tmp_path)


def test_lab_requests_the_file_lists_its_arguments_need(tmp_path):
    body = """
        var asked = _sent.filter(function(m){return m.type==='fs_list';});
        if (!asked.length || asked[0].kind !== 'logs')
            throw new Error('never asked for the logs listing');
        print('ok');
    """
    assert 'ok' in _run_lab(body, tmp_path)


def test_lab_collects_typed_arguments_including_awkward_paths(tmp_path):
    """Checkboxes must send booleans, and a quoted path must survive intact."""
    body = """
        _el('arg-log').value = "matlab/logs/it's b.jsonl";
        _el('arg-tagZ').value = '0.24';
        _el('arg-gapWeights').checked = false;
        _el('arg-est').value = 'mhe';
        Lab.run();
        var run = _sent[_sent.length-1];
        if (run.type !== 'job_run' || run.kind !== 'replay_rtls')
            throw new Error('bad job_run');
        if (run.args.gapWeights !== false) throw new Error('bool not sent as bool');
        if (run.args.tagZ !== '0.24' || run.args.est !== 'mhe')
            throw new Error('args not collected: '+JSON.stringify(run.args));
        if (run.args.log.indexOf("it's") < 0) throw new Error('quoted path mangled');
        print('ok');
    """
    assert 'ok' in _run_lab(body, tmp_path)


def test_lab_shows_artifacts_and_opens_the_report_automatically(tmp_path):
    body = """
        Lab.run();
        var req = _sent[_sent.length-1].req_id;
        Jobs.on({type:'job_started', job_id:'j9', kind:'replay_rtls',
                 title:'Replay', req_id:req, cancellable:true});
        if (Lab.jobId !== 'j9') throw new Error('LAB did not adopt its own job');
        Jobs.on({type:'job_done', job_id:'j9', elapsed_s:2.5, result:{},
                 artifacts:[{path:'matlab/results/x/report.txt',kind:'txt'},
                            {path:'matlab/results/x/traj.png',kind:'png'}]});
        var tabs = _el('lab-tabs');
        if (tabs.children.length !== 2) throw new Error('expected two artifact tabs');
        if (tabs.children[0].textContent !== 'report.txt') throw new Error('bad tab label');
        var r = _sent[_sent.length-1];
        if (r.type !== 'job_artifact_req' || r.path.indexOf('report.txt') < 0)
            throw new Error('did not auto-open the text report');
        tabs.children[1]._on.click();
        if (_sent[_sent.length-1].path.indexOf('.png') < 0)
            throw new Error('clicking a tab did not request that artifact');
        print('ok');
    """
    assert 'ok' in _run_lab(body, tmp_path)


def test_report_text_is_inserted_as_text_never_as_markup(tmp_path):
    """A report is file content; it must not be able to script the page."""
    body = """
        Lab.run();
        var req = _sent[_sent.length-1].req_id;
        Jobs.on({type:'job_started', job_id:'j9', kind:'replay_rtls', title:'R',
                 req_id:req});
        Jobs.on({type:'job_artifact', job_id:'j9', path:'x/report.txt', kind:'txt',
                 text:'RMSE 46 mm <img src=x onerror=alert(1)>'});
        if (_el('lab-out').innerHTML) throw new Error('report body became markup');
        if (_el('lab-out').textContent.indexOf('RMSE 46 mm') < 0)
            throw new Error('report not displayed');
        print('ok');
    """
    assert 'ok' in _run_lab(body, tmp_path)


def test_lab_ignores_jobs_it_did_not_start(tmp_path):
    """A calibration job running in another panel must not steal the pane."""
    body = """
        Lab.run();
        var req = _sent[_sent.length-1].req_id;
        Jobs.on({type:'job_started', job_id:'j9', kind:'replay_rtls', title:'R',
                 req_id:req});
        Jobs.on({type:'job_started', job_id:'other', kind:'hwcalib', title:'C',
                 req_id:'zzz'});
        if (Lab.jobId !== 'j9') throw new Error('LAB adopted an unrelated job');
        print('ok');
    """
    assert 'ok' in _run_lab(body, tmp_path)


# ── CAL mode ──────────────────────────────────────────────────────────────────

def test_all_eight_cal_step_panels_exist():
    """A Phase 2 regex deleted cal-5/6/7 (OK?/VISION/REG) and shipped that way.

    Their JS modules and server handlers survived, so the Kinect registration
    flow still worked — it just had no UI. Every step button must have a panel.
    """
    html = GUI.read_text()
    for n in range(1, 9):
        assert f'id="cal-{n}"' in html, f"CAL step {n} has no panel"
        assert f'id="cs-{n}"' in html, f"CAL step {n} has no button"


def test_the_markup_divs_balance():
    """The same regex also ate the CAL panel's closing tag.

    An unclosed div makes the browser nest every LATER panel inside the CAL
    panel, which is display:none outside CAL mode — so the JOBS and CONNECTION
    panels silently vanished in OPERATE/TUNE/DIAG/LAB. Nothing errors; they are
    just gone.
    """
    body = GUI.read_text().split('<script>')[0]
    assert body.count('<div') == body.count('</div>'), (
        f"{body.count('<div')} open vs {body.count('</div>')} close")


def test_cal_step_switching_tolerates_a_missing_panel():
    """It used to dereference every panel and throw on null."""
    js = _script()
    seg = js[js.index('function calStep('):js.index('function calStep(') + 700]
    assert 'if (pane)' in seg and 'if (btn)' in seg, seg[:300]


def test_cal_tools_are_generated_from_the_advertised_cal_jobs(tmp_path):
    body = """
        Jobs.on({type:'job_kinds', kinds:[
          {kind:'anchor_delay_push', title:'Push antenna delays', mode:'cal',
           args_schema:{ids:{t:'string',default:''},
                        ticks:{t:'number',default:16434},
                        apply:{t:'bool',default:false}}},
          {kind:'replay_rtls', title:'Replay', mode:'lab', args_schema:{}}]});

        if (CalTools.kind !== 'anchor_delay_push') throw new Error('no CAL tool selected');
        if (_el('cal-kind').innerHTML.indexOf('replay_rtls') >= 0)
            throw new Error('a LAB tool leaked into the CAL picker');

        var html = _el('cal-args').innerHTML;
        ['cal-arg-ids','cal-arg-ticks','cal-arg-apply'].forEach(function(id){
            if (html.indexOf(id) < 0) throw new Error('missing '+id); });

        // apply defaults OFF: this writes persistent state into hardware.
        if (html.indexOf('checked') >= 0)
            throw new Error('apply was pre-ticked');

        _el('cal-arg-ids').value = '1 2';
        _el('cal-arg-apply').checked = true;
        CalTools.run();
        var run = _sent[_sent.length-1];
        if (run.kind !== 'anchor_delay_push') throw new Error('wrong job run');
        if (run.args.ids !== '1 2' || run.args.apply !== true)
            throw new Error('args not collected: '+JSON.stringify(run.args));
        print('ok');
    """
    assert 'ok' in _run_lab(body, tmp_path)


def test_lab_and_cal_forms_do_not_collide(tmp_path):
    """Both are generated; distinct id prefixes keep them independent."""
    body = """
        Jobs.on({type:'job_kinds', kinds:[
          {kind:'a_cal', title:'C', mode:'cal', args_schema:{ticks:{t:'number',default:1}}},
          {kind:'a_lab', title:'L', mode:'lab', args_schema:{ticks:{t:'number',default:2}}}]});
        _el('cal-arg-ticks').value = '11';
        _el('arg-ticks').value = '22';
        if (collectArgs('a_cal','cal-arg-').ticks !== '11') throw new Error('cal args crossed');
        if (collectArgs('a_lab','arg-').ticks !== '22') throw new Error('lab args crossed');
        print('ok');
    """
    assert 'ok' in _run_lab(body, tmp_path)
