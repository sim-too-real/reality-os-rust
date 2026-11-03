$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..\..\..')).Path
$freezePath = Join-Path $PSScriptRoot 'FREEZE.json'
$resultPath = Join-Path $PSScriptRoot 'RESULT.json'
$reportPath = Join-Path $PSScriptRoot 'VERIFICATION.json'

function Assert-Condition([bool]$Condition, [string]$Message) {
    if (-not $Condition) { throw $Message }
}

function Get-Sha256([string]$Path) {
    return (Get-FileHash -LiteralPath $Path -Algorithm SHA256).Hash.ToUpperInvariant()
}

function Get-FiniteVector($Vector, [int]$Length, [string]$Name) {
    Assert-Condition ($null -ne $Vector -and $Vector.Count -eq $Length) "$Name must contain $Length values"
    $values = @()
    foreach ($value in $Vector) {
        Assert-Condition ($null -ne $value) "$Name contains a null value"
        $number = [double]$value
        Assert-Condition (-not [double]::IsNaN($number) -and -not [double]::IsInfinity($number)) "$Name contains a non-finite value"
        $values += $number
    }
    return $values
}

function Assert-VectorClose($Actual, [double[]]$Expected, [string]$Name) {
    $actualValues = Get-FiniteVector $Actual $Expected.Length $Name
    for ($i = 0; $i -lt $Expected.Length; $i++) {
        Assert-Condition ([Math]::Abs($actualValues[$i] - $Expected[$i]) -le 1e-8) "$Name component $i differs: actual=$($actualValues[$i]) expected=$($Expected[$i])"
    }
}

function Rotate-ContactVector([double[]]$Vector, [double[]]$Frame) {
    $world = @(0.0, 0.0, 0.0)
    for ($axis = 0; $axis -lt 3; $axis++) {
        $world[$axis] = $Vector[0] * $Frame[$axis] + $Vector[1] * $Frame[3 + $axis] + $Vector[2] * $Frame[6 + $axis]
    }
    return $world
}

$freezeBytes = [System.IO.File]::ReadAllBytes($freezePath)
$freezeHash = Get-Sha256 $freezePath
$freeze = Get-Content -LiteralPath $freezePath -Raw | ConvertFrom-Json
$result = Get-Content -LiteralPath $resultPath -Raw | ConvertFrom-Json
Assert-Condition ($freeze.status -eq 'FROZEN_BEFORE_SIMULATION') 'freeze was not marked before simulation'
Assert-Condition ($freeze.simulation_started -eq $false) 'freeze says simulation had already started'
Assert-Condition ($result.schema -eq 'realityos.grasp_lift_replay/v1') 'unexpected result schema'
Assert-Condition ($result.stage -eq 'candidate_primary') 'unexpected replay stage'
Assert-Condition ($result.freeze_sha256 -eq $freezeHash) 'result does not bind to this freeze'
Assert-Condition ($result.records.Count -eq 2 -and $freeze.primary_control_inputs.Count -eq 2) 'expected exactly two frozen trials'

$sourcePaths = @{
    integration_test_sha256 = 'crates/verify/tests/manipulation_benchmark.rs'
    execution_sha256 = 'crates/verify/src/manipulation_benchmark/execution.rs'
    reasoner_sha256 = 'crates/semantics/src/physical_reasoner.rs'
    worker_sha256 = 'crates/verify/mujoco_worker.py'
    observation_sha256 = 'crates/verify/src/observation.rs'
}
foreach ($key in $sourcePaths.Keys) {
    $path = Join-Path $repoRoot $sourcePaths[$key]
    Assert-Condition ((Get-Sha256 $path) -eq $freeze.candidate_source.$key) "source hash mismatch: $($sourcePaths[$key])"
}
$baselinePath = Join-Path $repoRoot $freeze.baseline_result.path
Assert-Condition ((Get-Sha256 $baselinePath) -eq $freeze.baseline_result.sha256) 'frozen baseline result hash mismatch'
$freezeRelativePath = 'docs/superpowers/evidence/grasp-lift-wrench-trace-20261103/FREEZE.json'
$trackedFreeze = & git -C $repoRoot ls-files --error-unmatch $freezeRelativePath 2>$null
Assert-Condition ($LASTEXITCODE -eq 0 -and $trackedFreeze) 'pre-run freeze is not committed'
$freezeCommit = (& git -C $repoRoot log -1 --format=%H -- $freezeRelativePath).Trim()
$freezeCommitUtc = [DateTimeOffset]::Parse((& git -C $repoRoot log -1 --format=%cI -- $freezeRelativePath).Trim()).UtcDateTime
$resultWriteUtc = (Get-Item -LiteralPath $resultPath).LastWriteTimeUtc
Assert-Condition ($freezeCommitUtc -le $resultWriteUtc) 'freeze commit is later than the replay result file'

$checks = [System.Collections.Generic.List[string]]::new()
$checks.Add('committed pre-run freeze, result, all five source files, baseline, and input hashes match')
$trialReports = @()
foreach ($inputBinding in $freeze.primary_control_inputs) {
    $record = @($result.records | Where-Object { $_.episode.trial_id -eq $inputBinding.trial_id })
    Assert-Condition ($record.Count -eq 1) "missing or duplicate result for $($inputBinding.trial_id)"
    $record = $record[0]
    $episode = $record.episode
    $inputPath = Join-Path $repoRoot $inputBinding.path
    Assert-Condition ((Get-Sha256 $inputPath) -eq $inputBinding.sha256) "input hash mismatch: $($inputBinding.trial_id)"
    Assert-Condition ($episode.trial_digest -eq $inputBinding.trial_digest) "trial digest mismatch: $($inputBinding.trial_id)"
    Assert-Condition ($episode.outcome -eq $inputBinding.historical_outcome) "outcome changed for $($inputBinding.trial_id)"
    Assert-Condition ($episode.failure_cause -eq $inputBinding.historical_failure_cause) "failure cause changed for $($inputBinding.trial_id)"
    Assert-Condition ($episode.unauthorized_writes -eq 0) "unauthorized writes detected for $($inputBinding.trial_id)"

    $phases = @($episode.grasp_contact_audits | Select-Object -ExpandProperty phase -Unique)
    Assert-Condition ($phases -contains 'post_close') "post_close trace missing for $($inputBinding.trial_id)"
    Assert-Condition (@($phases | Where-Object { $_ -like 'lift_waypoint_*' }).Count -gt 0) "lift trace missing for $($inputBinding.trial_id)"
    $contactCount = 0
    $verifiedWrenches = 0
    foreach ($audit in $episode.grasp_contact_audits) {
        [void](Get-FiniteVector $audit.object_com_world_m 3 'object_com_world_m')
        Assert-Condition ($audit.ctrl.Count -gt 0 -and $audit.actuator_force.Count -gt 0) "control/actuator trace missing for $($inputBinding.trial_id) phase $($audit.phase)"
        [void](Get-FiniteVector $audit.ctrl $audit.ctrl.Count 'ctrl')
        [void](Get-FiniteVector $audit.actuator_force $audit.actuator_force.Count 'actuator_force')
        foreach ($contact in $audit.contact_wrenches) {
            $contactCount++
            $derived = @($contact.force_world_on_object_n, $contact.torque_world_on_object_nm, $contact.moment_world_about_object_com_nm)
            if (-not $contact.force_available) {
                Assert-Condition (@($derived | Where-Object { $null -ne $_ }).Count -eq 0) 'unavailable contact wrench produced derived evidence'
                continue
            }
            $position = Get-FiniteVector $contact.position_world_m 3 'position_world_m'
            $com = Get-FiniteVector $audit.object_com_world_m 3 'object_com_world_m'
            $localForce = Get-FiniteVector $contact.force_local 3 'force_local'
            $localTorque = Get-FiniteVector $contact.torque_local 3 'torque_local'
            $frame = Get-FiniteVector $contact.contact_frame 9 'contact_frame'
            if ($contact.body1 -eq $episode.object_id) { $sign = -1.0 }
            elseif ($contact.body2 -eq $episode.object_id) { $sign = 1.0 }
            else { throw "contact does not include object body $($episode.object_id)" }
            $expectedForce = Rotate-ContactVector $localForce $frame
            $expectedTorque = Rotate-ContactVector $localTorque $frame
            for ($axis = 0; $axis -lt 3; $axis++) {
                $expectedForce[$axis] *= $sign
                $expectedTorque[$axis] *= $sign
            }
            $arm = @(0.0, 0.0, 0.0)
            for ($axis = 0; $axis -lt 3; $axis++) { $arm[$axis] = $position[$axis] - $com[$axis] }
            $momentX = ($arm[1] * $expectedForce[2]) - ($arm[2] * $expectedForce[1]) + $expectedTorque[0]
            $momentY = ($arm[2] * $expectedForce[0]) - ($arm[0] * $expectedForce[2]) + $expectedTorque[1]
            $momentZ = ($arm[0] * $expectedForce[1]) - ($arm[1] * $expectedForce[0]) + $expectedTorque[2]
            $expectedMoment = [double[]]@($momentX, $momentY, $momentZ)
            Assert-VectorClose $contact.force_world_on_object_n $expectedForce 'force_world_on_object_n'
            Assert-VectorClose $contact.torque_world_on_object_nm $expectedTorque 'torque_world_on_object_nm'
            Assert-VectorClose $contact.moment_world_about_object_com_nm $expectedMoment 'moment_world_about_object_com_nm'
            $verifiedWrenches++
        }
    }
    Assert-Condition ($contactCount -gt 0 -and $verifiedWrenches -gt 0) "no auditable contact wrench found for $($inputBinding.trial_id)"
    $trialReports += [ordered]@{
        trial_id = $episode.trial_id
        outcome = $episode.outcome
        failure_cause = $episode.failure_cause
        hold_verified = $episode.hold_verified
        unauthorized_writes = $episode.unauthorized_writes
        audit_rows = $episode.grasp_contact_audits.Count
        phases = $phases
        contacts = $contactCount
        derived_wrenches_recomputed = $verifiedWrenches
    }
}
$checks.Add('post_close and lift phases contain finite COM, ctrl, actuator, and contact fields')
$checks.Add('world force, signed torque, moment arm, and missing-wrench behavior independently recomputed')
$checks.Add('historical outcomes, failure causes, and zero unauthorized writes preserved')

$report = [ordered]@{
    schema = 'realityos.grasp_lift_wrench_trace_verification/v1'
    verified_at_utc = [DateTime]::UtcNow.ToString('yyyy-MM-ddTHH:mm:ssZ')
    freeze_sha256 = $freezeHash
    result_sha256 = Get-Sha256 $resultPath
    verifier_sha256 = Get-Sha256 $PSCommandPath
    freeze_commit = $freezeCommit
    freeze_commit_utc = $freezeCommitUtc.ToString('yyyy-MM-ddTHH:mm:ssZ')
    candidate_source = $freeze.candidate_source
    checks = $checks
    trials = $trialReports
    interpretation = 'Simulation-only diagnostic trace; solver forces are not a hardware-capacity measurement.'
}
$json = $report | ConvertTo-Json -Depth 12
[System.IO.File]::WriteAllText($reportPath, $json, [System.Text.UTF8Encoding]::new($false))
Write-Output "VERIFICATION_OK freeze=$freezeHash result=$($report.result_sha256) report=$reportPath"
Write-Output ($trialReports | ConvertTo-Json -Depth 8)
